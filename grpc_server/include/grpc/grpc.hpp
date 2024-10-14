// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <grpc++/grpc++.h>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "sample.grpc.pb.h"
// #include "x3.pb.h"
#include "x3.grpc.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace tros {

using grpc::ServerAsyncResponseWriter;
using grpc::ServerCompletionQueue;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using sample::SampleService;
using SampleRequest = x3::Capture;
using SampleResponse = x3::SmartFrameMessage;

struct compare_msg {
  bool operator()(const ai_msgs::msg::PerceptionTargets::SharedPtr m1,
                  const ai_msgs::msg::PerceptionTargets::SharedPtr m2) {
    return ((m1->header.stamp.sec > m2->header.stamp.sec) ||
            ((m1->header.stamp.sec == m2->header.stamp.sec) &&
             (m1->header.stamp.nanosec > m2->header.stamp.nanosec)));
  }
};

using GrpcCbType = std::function<int(ServerContext* context, const SampleRequest* request, SampleResponse* response)>;

class SyncSampleServiceImpl final : public SampleService::Service {
 public:
  SyncSampleServiceImpl(const std::string& addr, GrpcCbType cb) {
    server_address = addr;
    grpc_cb_ = cb;
  }

  void Init() {
    builder_.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder_.RegisterService(this);
    server_ = builder_.BuildAndStart();
  }

  void DeInit() {
    if (server_) {
      server_->Shutdown();  
    }
  }

  void Run(bool* running) {
    if (server_) {
      server_->Wait();
    }
  }

  Status SampleMethod(ServerContext* context, const SampleRequest* request, SampleResponse* response) override {
      if (grpc_cb_) {
        if (grpc_cb_(context, request, response) == 0) {
          return grpc::Status::OK;
        } else {
          // TODO
          // return grpc::Status::UNKNOWN;
          return grpc::Status::OK;
        }
      } else {
        const SampleRequest& cap = *request;
        printf("receive protobuf message timestamp: %ld, img w:%d, img h %d",
          cap.timestamp_(), cap.img_().width_(), cap.img_().height_());
        return Status::OK;
      }
      
      return Status::OK;
  }

 private:
  // Build server
  ServerBuilder builder_;
  std::unique_ptr<Server> server_ = nullptr;
  
  std::string server_address{"localhost:2511"};
  GrpcCbType grpc_cb_ = nullptr;
};


class AsyncServiceImpl final {
public:
    AsyncServiceImpl(const std::string& addr, GrpcCbType cb) {
      server_address = addr;
      grpc_cb_ = cb;
    }

    ~AsyncServiceImpl() {
    }

    void Init() {
      // Run server
      std::cout << "Server listening on " << server_address << std::endl;
      builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
      builder.RegisterService(&_service);
      _queue = builder.AddCompletionQueue();
      _server = builder.BuildAndStart();
    }

    void DeInit() {
      _server->Shutdown();
      _queue->Shutdown();
    }

    void Run(bool* running) {
      HandleRPCs(running);
    }

private:
    class CallData {
    public:
        CallData(SampleService::AsyncService* service, ServerCompletionQueue* queue)
                : _service{service}, _queue{queue}, _responder{&_context}, _status{CallStatus::CREATE} {
            Proceed();
        }

        void SetCallback(GrpcCbType cb) {
          _grpc_cb = cb;
        }

        void Proceed() {
            switch (_status) {
                case CallStatus::CREATE: {
                    _status = CallStatus::PROCESS;
                    _service->RequestSampleMethod(&_context, &_request, &_responder, _queue, _queue, this);
                    break;
                }
                case CallStatus::PROCESS: {
                    auto call_data = new CallData{_service, _queue};
                    call_data->SetCallback(_grpc_cb);
                    auto status = grpc::Status::OK;
                    
                    if (_grpc_cb) {
                      if (_grpc_cb(&_context, &_request, &_response) == 0) {
                        status = grpc::Status::OK;
                      } else {
                        // TODO
                        // return grpc::Status::UNKNOWN;
                        status = grpc::Status::OK;
                      }
                    } else {
                      const SampleRequest& cap = _request;
                      printf("receive protobuf message timestamp: %ld, img w:%d, img h %d",
                        cap.timestamp_(), cap.img_().width_(), cap.img_().height_());
                    }
      
                    _status = CallStatus::FINISH;
                    _responder.Finish(_response, status, this);
                    break;
                }
                default: {
                    delete this;
                }
            }
        }

    private:
        SampleService::AsyncService* _service;
        ServerCompletionQueue* _queue;
        ServerContext _context;
        SampleRequest _request;
        SampleResponse _response;
        ServerAsyncResponseWriter<SampleResponse> _responder;
        enum class CallStatus {
            CREATE, PROCESS, FINISH
        };
        CallStatus _status;
        GrpcCbType _grpc_cb = nullptr;
    };

    void HandleRPCs(bool* running) {
        auto call_data = new CallData{&_service, _queue.get()};
        call_data->SetCallback(grpc_cb_);
        void* tag;
        bool ok;
        while (*running) {
            if (_queue->Next(&tag, &ok) && ok) {
                static_cast<CallData*>(tag)->Proceed();
            } else {
                std::cerr << "Something went wrong" << std::endl;
                abort();
            }
        }
    }

    // Build server
    ServerBuilder builder;
    SampleService::AsyncService _service;
    std::unique_ptr<ServerCompletionQueue> _queue;
    std::unique_ptr<Server> _server;
    GrpcCbType grpc_cb_ = nullptr;
    std::string server_address{"localhost:2511"};
};


class Grpc : public rclcpp::Node {
 public:
  Grpc(std::string node_name) : rclcpp::Node(node_name) {
    smart_topic_name_ = this->declare_parameter("smart_topic", smart_topic_name_);
    ros_publisher_topic_ = this->declare_parameter("ros_publisher_topic", ros_publisher_topic_);
    server_address_ = this->declare_parameter("server_address", server_address_);
    is_sync_mode_ = this->declare_parameter("is_sync_mode", is_sync_mode_);

    RCLCPP_WARN_STREAM(this->get_logger(),
                        "\nParameter:"
                        << "\n smart_topic: " << smart_topic_name_
                        << "\n ros_publisher_topic: " << ros_publisher_topic_
                        << "\n server_address: " << server_address_
                        << "\n is_sync_mode: " << is_sync_mode_);

    server_thread_ = std::make_unique<std::thread>([this] {
      if (is_sync_mode_) {
        sync_server_ = std::make_unique<SyncSampleServiceImpl>(server_address_,
          std::bind(&Grpc::OnGetGrpcRequest,
          this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sync_server_->Init();
        sync_server_->Run(&running_);
      } else {
        async_server_ = std::make_unique<AsyncServiceImpl>(server_address_, std::bind(&Grpc::OnGetGrpcRequest,
          this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        async_server_->Init();
        async_server_->Run(&running_);
      }
      RCLCPP_WARN(this->get_logger(), "Server thread exited");
    });

    ai_msg_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        smart_topic_name_,
        10,
        std::bind(&Grpc::OnGetSmartMessage, this, std::placeholders::_1));

    ros_publisher_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        ros_publisher_topic_, 10);

  }

  ~Grpc() {
    RCLCPP_WARN(this->get_logger(), "Grpc destructor called");
    running_ = false;
    if (is_sync_mode_ && sync_server_) {
      sync_server_->DeInit();
    } else if (async_server_) {
      async_server_->DeInit();
    }

    if (server_thread_ && server_thread_->joinable()) {
      server_thread_->join();
    }
  }
  
 private:
  std::string server_address_{"localhost:2510"};
  int is_sync_mode_ = 1;
  std::string smart_topic_name_ = "/hobot_mono2d_body_detection";
  std::string ros_publisher_topic_ = "/image";

  std::unique_ptr<SyncSampleServiceImpl> sync_server_ = nullptr;
  std::unique_ptr<AsyncServiceImpl> async_server_ = nullptr;

  std::unique_ptr<std::thread> server_thread_ = nullptr;
  
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ros_publisher_compressed_ =
      nullptr;

  std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>,
                      compare_msg>
      x3_smart_msg_;

  std::mutex map_smart_mutex_;
  std::condition_variable map_smart_condition_;
  bool running_ = true;

  void OnGetSmartMessage(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "OnGetSmartMessage ts: %d.%d",
      msg->header.stamp.sec, msg->header.stamp.nanosec);
    {
      std::unique_lock<std::mutex> lock(map_smart_mutex_);
      x3_smart_msg_.push(msg);
      if (x3_smart_msg_.size() > 100) {
        x3_smart_msg_.pop();
        RCLCPP_WARN(this->get_logger(),
                    "cache smart message num > 100, drop the "
                    "oldest smart message");
      }
      map_smart_condition_.notify_one();
    }

    return;
  }
  
  int OnGetGrpcRequest(ServerContext* context, const SampleRequest* request, SampleResponse* response) {
    const SampleRequest& cap = *request;
    RCLCPP_INFO(get_logger(), "receive protobuf message timestamp: %ld, img w:%d, img h %d",
      cap.timestamp_(), cap.img_().width_(), cap.img_().height_());
    // TODO for debug
    if (0)
    {
      std::string file_name = "./ws_" + std::to_string(cap.timestamp_()) + ".jpg";
      std::ofstream ofs(file_name, std::ios::binary);
      ofs << cap.img_().buf_();
    }

    sensor_msgs::msg::CompressedImage::UniquePtr msg(new sensor_msgs::msg::CompressedImage());
    msg->header.stamp.sec = cap.timestamp_() / 1000000000;
    msg->header.stamp.nanosec = cap.timestamp_() - msg->header.stamp.sec * 1000000000;
    msg->header.frame_id = "default_cam";
    msg->format = "jpeg";
    msg->data.resize(cap.img_().buf_().size());
    memcpy(&msg->data[0], cap.img_().buf_().data(), cap.img_().buf_().size());
    if (ros_publisher_compressed_) {
      ros_publisher_compressed_->publish(std::move(msg));
      RCLCPP_INFO(get_logger(), "publish ros2 message with topic %s", ros_publisher_topic_.data());

      ai_msgs::msg::PerceptionTargets::SharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(map_smart_mutex_);
        map_smart_condition_.wait_for(lock, std::chrono::milliseconds(1000),
          [&] { return !running_ || !x3_smart_msg_.empty(); });
        if (!running_ || x3_smart_msg_.empty()) {
          return -1;
        }
        msg = x3_smart_msg_.top();
        x3_smart_msg_.pop();
      }

      SampleResponse& frame_msg = *response;
      FrameAddSmart(frame_msg, msg);
      
      const auto& smart = frame_msg;
      std::stringstream ss;
      ss << "recved smart message ts: " << smart.timestamp_()
        << ", error_code: " << smart.error_code_()
        << ", target size: " << smart.targets_().size();
      for (const auto& target : smart.targets_()) {
        ss << "\n type: " << target.type_();
        for (const auto& box : target.boxes_()) {
          ss << "\n\t box type: " << box.type_()
            << ", top_left: " << box.top_left_().x_() << " " << box.top_left_().y_()
            << ", bottom_right: " << box.bottom_right_().x_() << " " << box.bottom_right_().y_()
            << ", score: " << box.score();
        }
        for (const auto& attr : target.attributes_()) {
          ss << "\n\t attr name: " << attr.type_() << ", val: " << attr.value_();
        }
      }

      RCLCPP_INFO(this->get_logger(), "%s", ss.str().data());

      return 0;
    } else {
      RCLCPP_ERROR(get_logger(), "ros publisher is not initialized");
      return -1;
    }
    return 0;
  }

  int FrameAddSmart(
      SampleResponse &smart,
      ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg) {
    smart.set_timestamp_(static_cast<uint64_t>(smart_msg->header.stamp.sec) * 1000000000 +
                          smart_msg->header.stamp.nanosec);
    smart.set_error_code_(0);
    for (auto smart_target : smart_msg->targets) {
      auto target = smart.add_targets_();
      target->set_track_id_(smart_target.track_id);
      target->set_type_(smart_target.type);

      if (smart_target.rois.size() == 1 &&
          !smart_target.rois.front().type.empty()) {
        target->set_type_(smart_target.rois.front().type);
      }

      // rois
      for (auto smart_roi : smart_target.rois) {
        auto proto_box = target->add_boxes_();
        proto_box->set_type_(smart_roi.type);
        auto point1 = proto_box->mutable_top_left_();
        point1->set_x_(smart_roi.rect.x_offset < 1 ? 1 : smart_roi.rect.x_offset);
        point1->set_y_(smart_roi.rect.y_offset < 1 ? 1 : smart_roi.rect.y_offset);
        auto point2 = proto_box->mutable_bottom_right_();
        point2->set_x_(smart_roi.rect.x_offset + smart_roi.rect.width);
        point2->set_y_(smart_roi.rect.y_offset + smart_roi.rect.height);
        proto_box->set_score(smart_roi.confidence);
      }

      // attributes
      for (auto smart_attributes : smart_target.attributes) {
        auto attrs = target->add_attributes_();
        attrs->set_type_(smart_attributes.type);
        attrs->set_value_(smart_attributes.value);
        // attrs->set_score_(1.0);
        if (smart_attributes.type == "gender") {
          if (smart_attributes.value == 1) {
            attrs->set_value_string_("男");
          } else if (smart_attributes.value == -1) {
            attrs->set_value_string_("女");
          } else {
            attrs->set_value_string_(std::to_string(smart_attributes.value));
          }
        } else {
          attrs->set_value_string_(std::to_string(smart_attributes.value));
        }
      }

      // points
      for (auto smart_points : smart_target.points) {
        auto proto_points = target->add_points_();
        std::string pt_type = "body_kps";
        if (smart_points.type == "body_kps") {
          pt_type = "body_landmarks";
        } else if (smart_points.type == "hand_kps") {
          pt_type = "hand_landmarks";
        } else if (smart_points.type == "face_kps") {
          pt_type = "lmk_106pts";
        } else {
          pt_type = smart_points.type;
        }

        proto_points->set_type_(pt_type);
        for (auto smart_point : smart_points.point) {
          auto point = proto_points->add_points_();
          point->set_x_(smart_point.x);
          point->set_y_(smart_point.y);
          point->set_score_(1.0);
        }
      }
    }

    return 0;
  }

};

}
