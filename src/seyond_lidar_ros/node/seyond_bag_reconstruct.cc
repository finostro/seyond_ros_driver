// Offline rebuild of point clouds from a bag of recorded seyond packets.
//
//   ros2 run seyond seyond_bag_reconstruct <input_bag> [options]
//
// Reads every seyond/msg/SeyondScan topic in the input bag, converts the packets with the
// same driver code as the live node and writes one sensor_msgs/PointCloud2 per frame.
//
// The HV table needed by compact packets is only sent every 20 frames, so a recording
// usually starts with packets that come before the first table. A first, short pass finds
// the first table of each lidar and uses it for those earlier packets ("backwards in
// time"); later tables found in the bag replace it from that point on.
//
// Each lidar is decoded on its own thread; the bag is read once more in a single
// sequential pass.

#include <rcutils/time.h>
#include <rmw/rmw.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "driver_lidar.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "seyond/msg/seyond_scan.hpp"
#include "yaml_tools.hpp"

namespace {

using BagMsg = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;
constexpr const char *kScanType = "seyond/msg/SeyondScan";
constexpr const char *kCloudType = "sensor_msgs/msg/PointCloud2";

std::string strip_slash(const std::string &s) {
  size_t i = s.find_first_not_of('/');
  return i == std::string::npos ? "" : s.substr(i);
}

bool deserialize_scan(const BagMsg &bag_msg, seyond::msg::SeyondScan &out) {
  static const rosidl_message_type_support_t *ts =
      rosidl_typesupport_cpp::get_message_type_support_handle<seyond::msg::SeyondScan>();
  return rmw_deserialize(bag_msg->serialized_data.get(), ts, &out) == RMW_RET_OK;
}

// Blocking queue with a capacity, so the reader cannot run far ahead of the decoders.
class BoundedQueue {
 public:
  explicit BoundedQueue(size_t cap) : cap_(cap) {}
  void push(BagMsg m) {
    std::unique_lock<std::mutex> lk(mu_);
    not_full_.wait(lk, [&] { return q_.size() < cap_; });
    q_.push_back(std::move(m));
    not_empty_.notify_one();
  }
  // nullptr means "no more messages"
  BagMsg pop() {
    std::unique_lock<std::mutex> lk(mu_);
    not_empty_.wait(lk, [&] { return !q_.empty(); });
    BagMsg m = std::move(q_.front());
    q_.pop_front();
    not_full_.notify_one();
    return m;
  }

 private:
  size_t cap_;
  std::deque<BagMsg> q_;
  std::mutex mu_;
  std::condition_variable not_full_, not_empty_;
};

struct LidarJob {
  std::string in_topic;
  std::string out_topic;
  seyond::LidarConfig cfg;
  std::unique_ptr<seyond::DriverLidar> driver;
  BoundedQueue queue{64};
  std::thread thread;
  sensor_msgs::msg::PointCloud2 cloud_template;
  // byte ranges of a point that no field covers (struct padding), zeroed on output
  std::vector<std::pair<uint32_t, uint32_t>> padding;

  // current frame
  bool frame_open = false;
  bool frame_valid = true;
  bool first_frame = true;
  uint64_t frame_idx = 0;
  double frame_ts_us = 0.0;
  rcutils_time_point_value_t last_recv = 0;

  // stats
  uint64_t scans = 0, packets = 0, bad_packets = 0, frames_written = 0;
  uint64_t frames_no_table = 0, frames_missing_end = 0, frames_empty = 0;
  uint64_t tables_seen = 0;
};

class Reconstructor {
 public:
  Reconstructor(rosbag2_cpp::Writer &writer) : writer_(writer) {}

  void run_job(LidarJob &job) {
    seyond::msg::SeyondScan scan;
    while (BagMsg bag_msg = job.queue.pop()) {
      if (!deserialize_scan(bag_msg, scan)) {
        fprintf(stderr, "[%s] failed to deserialize a SeyondScan, skipped\n", job.in_topic.c_str());
        continue;
      }
      job.scans++;
      handle_scan(job, scan, bag_msg->recv_timestamp);
    }
    // the bag usually ends in the middle of a frame: that partial frame is dropped
  }

 private:
  void handle_scan(LidarJob &job, const seyond::msg::SeyondScan &scan, rcutils_time_point_value_t recv) {
    auto &drv = *job.driver;
    for (const auto &pkt : scan.packets) {
      job.packets++;
      if (pkt.has_table && !pkt.table.empty()) {
        drv.anglehv_table_.assign(pkt.table.begin(), pkt.table.end());
        drv.anglehv_table_init_ = true;
        job.tables_seen++;
      }
      if (pkt.data.size() < sizeof(InnoDataPacket)) {
        job.bad_packets++;
        continue;
      }
      const auto *p = reinterpret_cast<const InnoDataPacket *>(pkt.data.data());
      if (pkt.data.size() < sizeof(InnoDataPacket) + uint64_t(p->item_number) * p->item_size) {
        job.bad_packets++;
        continue;
      }

      if (job.frame_open && p->idx != job.frame_idx) {
        // a new frame started without an is_last_scan: that scan message was lost
        job.frames_missing_end++;
        emit_frame(job);
      }
      if (!job.frame_open) {
        job.frame_open = true;
        job.frame_valid = true;
        job.frame_idx = p->idx;
        job.frame_ts_us = p->common.ts_start_us;
        // point timestamps are relative to the first packet of their frame, as in live mode
        drv.frame_start_ts_ = p->common.ts_start_us;
      }
      if (CHECK_CO_SPHERE_POINTCLOUD_DATA(p->type) && !drv.anglehv_table_init_) {
        job.frame_valid = false;  // no HV table anywhere in the bag for this lidar
        continue;
      }
      drv.convert_and_parse(reinterpret_cast<const int8_t *>(p));
    }
    job.last_recv = recv;
    if (scan.is_last_scan) {
      emit_frame(job);
    }
  }

  void emit_frame(LidarJob &job) {
    if (!job.frame_open) {
      return;
    }
    auto &cloud = *job.driver->pcl_pc_ptr;
    const bool first = job.first_frame;
    job.first_frame = false;
    job.frame_open = false;

    // The first frame of each lidar is partial: the recording (and the driver) start mid-frame.
    if (first) {
      cloud.clear();
      return;
    }
    if (!job.frame_valid) {
      job.frames_no_table++;
      cloud.clear();
      return;
    }
    if (cloud.points.empty()) {
      job.frames_empty++;
      return;
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    job.driver->transform_pointcloud();

    sensor_msgs::msg::PointCloud2 msg = job.cloud_template;
    const int64_t ts_ns = static_cast<int64_t>(job.frame_ts_us * 1000.0);
    msg.header.stamp.sec = static_cast<int32_t>(ts_ns / 1000000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(ts_ns % 1000000000);
    msg.header.frame_id = job.cfg.frame_id;
    msg.height = 1;
    msg.width = cloud.points.size();
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(size_t(msg.row_step));
    std::memcpy(msg.data.data(), cloud.points.data(), msg.data.size());
    // the driver leaves the padding of each point uninitialized: don't write that memory out
    for (uint8_t *pt = msg.data.data(), *end = pt + msg.data.size(); pt < end; pt += msg.point_step) {
      for (const auto &[off, len] : job.padding) std::memset(pt + off, 0, len);
    }
    cloud.clear();

    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    serializer_.serialize_message(&msg, serialized.get());
    writer_.write(serialized, job.out_topic, kCloudType, job.last_recv, job.last_recv);
    job.frames_written++;
  }

  rosbag2_cpp::Writer &writer_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer_;
};

void usage(const char *argv0) {
  fprintf(stderr,
          "Usage: %s <input_bag> [-o output_bag] [-c seyond_config.yaml] [--clouds-only]\n"
          "          [--preset <storage preset>]\n\n"
          "  -o            output bag (default: <input_bag>_clouds)\n"
          "  -c            driver config that maps packet topics to cloud topics, frame ids,\n"
          "                ranges and coordinate mode\n"
          "                (default: sample_collector_bringup/config/seyond_config.yaml)\n"
          "  --clouds-only only write the rebuilt clouds; by default every other topic is\n"
          "                copied too, except the packets and any cloud topic being rebuilt\n"
          "  --preset      mcap storage preset: zstd_fast (default), fastwrite (no compression), zstd_small, ...\n",
          argv0);
}

}  // namespace

int main(int argc, char **argv) {
  std::string in_uri, out_uri, config_path, preset = "zstd_fast";
  bool clouds_only = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage(argv[0]);
        exit(2);
      }
      return argv[++i];
    };
    if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else if (a == "-o") {
      out_uri = next();
    } else if (a == "-c") {
      config_path = next();
    } else if (a == "--preset") {
      preset = next();
    } else if (a == "--clouds-only") {
      clouds_only = true;
    } else if (!a.empty() && a[0] == '-') {
      fprintf(stderr, "unknown option %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    } else if (in_uri.empty()) {
      in_uri = a;
    } else {
      usage(argv[0]);
      return 2;
    }
  }
  if (in_uri.empty()) {
    usage(argv[0]);
    return 2;
  }
  while (in_uri.size() > 1 && in_uri.back() == '/') in_uri.pop_back();
  if (out_uri.empty()) {
    std::string base = in_uri;
    if (base.size() > 5 && base.compare(base.size() - 5, 5, ".mcap") == 0) base.resize(base.size() - 5);
    out_uri = base + "_clouds";
  }
  if (config_path.empty()) {
    try {
      config_path = ament_index_cpp::get_package_share_directory("sample_collector_bringup") +
                    "/config/seyond_config.yaml";
    } catch (const std::exception &) {
      fprintf(stderr, "sample_collector_bringup not found, pass the driver config with -c\n");
      return 2;
    }
  }

  std::string log_level = "warn";
  seyond::DriverLidar::init_log_s(log_level, [](int32_t level, const char *h, const char *m) {
    if (level <= 4) fprintf(stderr, "[seyond] %s %s\n", h ? h : "", m ? m : "");
  });

  std::vector<seyond::LidarConfig> cfgs;
  seyond::CommonConfig common;
  if (seyond::YamlTools::parseConfig(config_path, cfgs, common) != 0) {
    fprintf(stderr, "cannot parse %s\n", config_path.c_str());
    return 1;
  }

  // ---- match the bag's packet topics to the config
  rosbag2_cpp::Reader probe;
  probe.open(in_uri);
  const auto topics = probe.get_all_topics_and_types();
  const auto meta = probe.get_metadata();
  probe.close();

  std::map<std::string, rosbag2_storage::TopicMetadata> topic_meta;
  for (const auto &t : topics) topic_meta[t.name] = t;

  std::vector<std::unique_ptr<LidarJob>> jobs;
  std::map<std::string, LidarJob *> job_by_topic;
  std::set<std::string> rebuilt_topics;
  for (const auto &t : topics) {
    if (t.type != kScanType) continue;
    const std::string name = strip_slash(t.name);
    const seyond::LidarConfig *match = nullptr;
    std::string prefix;
    for (const auto &c : cfgs) {
      const std::string cp = strip_slash(c.packet_topic);
      if (name == cp || (name.size() > cp.size() && name.compare(name.size() - cp.size(), cp.size(), cp) == 0 &&
                         name[name.size() - cp.size() - 1] == '/')) {
        match = &c;
        prefix = name.substr(0, name.size() - cp.size());
        break;
      }
    }
    if (!match) {
      fprintf(stderr, "packet topic %s is not in %s, skipped\n", t.name.c_str(), config_path.c_str());
      continue;
    }
    auto job = std::make_unique<LidarJob>();
    job->in_topic = t.name;
    job->out_topic = "/" + prefix + strip_slash(match->frame_topic);
    job->cfg = *match;
    job->driver = std::make_unique<seyond::DriverLidar>(job->cfg);
    pcl::PointCloud<SeyondPoint> empty;
    pcl::toROSMsg(empty, job->cloud_template);
    {
      std::vector<bool> used(job->cloud_template.point_step, false);
      for (const auto &f : job->cloud_template.fields) {
        const uint32_t sz = ::sizeOfPointField(f.datatype) * std::max<uint32_t>(1, f.count);
        for (uint32_t b = f.offset; b < f.offset + sz && b < used.size(); ++b) used[b] = true;
      }
      for (uint32_t b = 0; b < used.size();) {
        if (used[b]) { ++b; continue; }
        uint32_t e = b;
        while (e < used.size() && !used[e]) ++e;
        job->padding.emplace_back(b, e - b);
        b = e;
      }
    }
    rebuilt_topics.insert(job->out_topic);
    job_by_topic[t.name] = job.get();
    jobs.push_back(std::move(job));
  }
  if (jobs.empty()) {
    fprintf(stderr, "no %s topics in %s that match %s\n", kScanType, in_uri.c_str(), config_path.c_str());
    return 1;
  }

  // ---- pass 1: first HV table of each lidar, used for the packets that come before it
  auto t0 = std::chrono::steady_clock::now();
  {
    std::set<std::string> pending;
    for (auto &j : jobs)
      if (!j->driver->anglehv_table_init_) pending.insert(j->in_topic);  // hv_table_file not set
    if (!pending.empty()) {
      rosbag2_cpp::Reader reader;
      reader.open(in_uri);
      rosbag2_storage::StorageFilter filter;
      filter.topics.assign(pending.begin(), pending.end());
      reader.set_filter(filter);
      seyond::msg::SeyondScan scan;
      while (!pending.empty() && reader.has_next()) {
        BagMsg m = reader.read_next();
        if (!pending.count(m->topic_name) || !deserialize_scan(m, scan)) continue;
        for (const auto &pkt : scan.packets) {
          if (pkt.has_table && !pkt.table.empty()) {
            auto &drv = *job_by_topic[m->topic_name]->driver;
            drv.anglehv_table_.assign(pkt.table.begin(), pkt.table.end());
            drv.anglehv_table_init_ = true;
            double at = (m->recv_timestamp - meta.starting_time.time_since_epoch().count()) * 1e-9;
            printf("%s: HV table found %.2f s into the bag\n", m->topic_name.c_str(), at);
            pending.erase(m->topic_name);
            break;
          }
        }
      }
      for (const auto &t : pending)
        fprintf(stderr, "WARNING %s: no HV table in the bag, its compact frames will be dropped\n", t.c_str());
    }
  }
  auto t1 = std::chrono::steady_clock::now();

  // ---- pass 2: decode and write
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions out_opts;
  out_opts.uri = out_uri;
  out_opts.storage_id = "mcap";
  out_opts.storage_preset_profile = preset;
  writer.open(out_opts, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

  for (auto &j : jobs) {
    rosbag2_storage::TopicMetadata tm;
    auto it = topic_meta.find(j->out_topic);
    if (it != topic_meta.end() && it->second.type == kCloudType) {
      tm = it->second;  // keep the QoS of the live cloud topic
    }
    tm.name = j->out_topic;
    tm.type = kCloudType;
    tm.serialization_format = "cdr";
    writer.create_topic(tm);
  }
  std::set<std::string> copied;
  if (!clouds_only) {
    for (const auto &t : topics) {
      if (job_by_topic.count(t.name) || rebuilt_topics.count(t.name)) continue;
      writer.create_topic(t);
      copied.insert(t.name);
    }
  }

  Reconstructor rec(writer);
  for (auto &j : jobs) {
    LidarJob *jp = j.get();
    j->thread = std::thread([&rec, jp] { rec.run_job(*jp); });
  }

  rosbag2_cpp::Reader reader;
  reader.open(in_uri);
  if (clouds_only) {
    rosbag2_storage::StorageFilter filter;
    for (auto &j : jobs) filter.topics.push_back(j->in_topic);
    reader.set_filter(filter);
  }
  const double bag_start = meta.starting_time.time_since_epoch().count() * 1e-9;
  const double bag_len = std::max(1e-9, std::chrono::duration<double>(meta.duration).count());
  uint64_t bytes = 0, copied_msgs = 0;
  auto last_print = std::chrono::steady_clock::now();
  while (reader.has_next()) {
    BagMsg m = reader.read_next();
    bytes += m->serialized_data->buffer_length;
    const double recv_s = m->recv_timestamp * 1e-9;
    auto it = job_by_topic.find(m->topic_name);
    if (it != job_by_topic.end()) {
      it->second->queue.push(std::move(m));
    } else if (copied.count(m->topic_name)) {
      writer.write(m);
      copied_msgs++;
    }
    auto now = std::chrono::steady_clock::now();
    if (now - last_print > std::chrono::seconds(2)) {
      last_print = now;
      double el = std::chrono::duration<double>(now - t1).count();
      double done = (recv_s - bag_start) / bag_len;
      fprintf(stderr, "\r%5.1f%%  %.0f MB/s   ", 100.0 * std::min(1.0, std::max(0.0, done)), bytes / 1e6 / el);
    }
  }
  for (auto &j : jobs) j->queue.push(nullptr);
  for (auto &j : jobs) j->thread.join();
  reader.close();
  writer.close();
  auto t2 = std::chrono::steady_clock::now();

  double el = std::chrono::duration<double>(t2 - t1).count();
  fprintf(stderr, "\r");
  printf("\nWrote %s (table scan %.1f s, decode %.1f s, %.0f MB/s, %.1fx real time)\n", out_uri.c_str(),
         std::chrono::duration<double>(t1 - t0).count(), el, bytes / 1e6 / el, bag_len / el);
  for (auto &j : jobs) {
    printf("  %s -> %s: %lu clouds", j->in_topic.c_str(), j->out_topic.c_str(), j->frames_written);
    if (j->frames_no_table) printf(", %lu dropped (no HV table)", j->frames_no_table);
    if (j->frames_missing_end) printf(", %lu split on a lost end-of-frame", j->frames_missing_end);
    if (j->frames_empty) printf(", %lu empty", j->frames_empty);
    if (j->bad_packets) printf(", %lu bad packets", j->bad_packets);
    printf("\n");
  }
  if (!clouds_only) printf("  %lu messages copied from %zu other topics\n", copied_msgs, copied.size());
  return 0;
}
