#include <memory>
#include <iostream>
#include <thread>
#include <signal.h>
#include "sample.grpc.pb.h"
#include <grpc++/grpc++.h>
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using grpc::ClientAsyncResponseReader;
using grpc::CompletionQueue;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using sample::SampleService;

using SampleRequest = x3::Capture;
using SampleResponse = x3::SmartFrameMessage;

bool is_running_ = true;
bool is_sync_mode = true;

// 信号处理函数  
void signalHandler(int signum) {  
    std::cout << "catch signal " << signum << std::endl;
    is_running_ = false;
}  

void ParseResponse(const SampleResponse& response) {
  const auto& smart = response;
  std::stringstream ss;
  ss << "Client response ts: " << smart.timestamp_()
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
  std::cout << ss.str() << std::endl;
}

class SyncSampleClient {
public:
    SyncSampleClient(std::shared_ptr<Channel> channel) : _stub{SampleService::NewStub(channel)} {}

    std::string SampleMethod(SampleRequest& request) {
        // Prepare request
        // request.set_request_sample_field(request_sample_field);

        request.set_timestamp_(std::chrono::system_clock::now().time_since_epoch().count());
        std::cout << "Client request ts: " << request.timestamp_() << std::endl;

        // Send request
        SampleResponse response;
        ClientContext context;
        Status status;
        status = _stub->SampleMethod(&context, request, &response);

        // Handle response
        if (status.ok()) {
          ParseResponse(response);
          return "success";
        } else {
            std::cerr << "fail!!! error_code: " << status.error_code() << ": " << status.error_message() << std::endl;
            return "RPC failed";
        }
    }

private:
    std::unique_ptr<SampleService::Stub> _stub;
    
    std::thread send_task_;
    bool is_running_ = true;
    bool is_connected_ = false;
};


class AsyncSampleClient {
public:
    AsyncSampleClient(std::shared_ptr<Channel> channel) : _stub{SampleService::NewStub(channel)} {}

    void SampleMethod(SampleRequest& request) {
        request.set_timestamp_(std::chrono::system_clock::now().time_since_epoch().count());
        std::cout << "Client request ts: " << request.timestamp_() << std::endl;

        // Create an AsyncClientCall object to store RPC data
        auto* call = new AsyncClientCall;
        
        // Create an RPC object
        call->rpc = _stub->PrepareAsyncSampleMethod(&call->context, request, &_queue);
        
        // Initiate the RPC call
        call->rpc->StartCall();
        
        // Request to update the server's response and the call status upon completion of the RPC
        call->rpc->Finish(&call->response, &call->status, (void*)call);
    }

    void AsyncCompleteRPC() {
        void* tag;
        bool ok = false;
        while (_queue.Next(&tag, &ok)) {
            if (!ok) {
                std::cerr << "Something went wrong" << std::endl;
                abort();
            }
            std::string err;
            auto* call = static_cast<AsyncClientCall*>(tag);
            if (call) {
                if (call->status.ok()) {
                    ParseResponse(call->response);
                } else {
                    std::cerr << "fail!!! error_code: " << call->status.error_code() << ": " << call->status.error_message() << std::endl;
                    std::cout << "Client received: RPC failed" << std::endl;
                }
            } else {
                err = "A client call was deleted";
            }
            delete call;
            if (!err.empty()) {
                throw std::runtime_error(err);
            }
        }
    }

private:
    struct AsyncClientCall {
        SampleResponse response;
        ClientContext context;
        Status status;
        std::unique_ptr<ClientAsyncResponseReader<SampleResponse>> rpc;
    };
    std::unique_ptr<SampleService::Stub> _stub;
    CompletionQueue _queue;
};


bool processImage(const std::string &image_source,
                  const std::string &image_format,
                  SampleRequest& capture) {
  if (access(image_source.c_str(), R_OK) == -1) {
    printf(
                 "Image: %s not exist!",
                 image_source.c_str());
    return false;
  }
  cv::Mat bgr_mat;

  // 获取图片
  if (image_format == "jpeg") {
    bgr_mat = cv::imread(image_source, cv::IMREAD_COLOR);
  }

  // 使用opencv的imencode接口将mat转成vector，获取图片size
  std::vector<int> param;
  std::vector<uint8_t> jpeg_data;
  imencode(".jpg", bgr_mat, jpeg_data, param);
  
  // 获取当前时间，转成纳秒时间戳
  capture.set_timestamp_(std::chrono::system_clock::now().time_since_epoch().count());
  auto image = capture.mutable_img_();
  image->set_buf_((const char *)jpeg_data.data(), jpeg_data.size());
  image->set_type_("jpeg");
  image->set_width_(bgr_mat.cols);
  image->set_height_(bgr_mat.rows);

  return true;
}

int main(int argc, char** argv) {
  // 注册信号SIGINT和对应的处理函数  
  signal(SIGINT, signalHandler);    

  if (argc < 3) {
    std::cout << "Usage: ./sample_client <url> <img file> <is sync mode>" << std::endl;
    std::cout << "\t e.g.: ./sample_client localhost:2510 ./test.jpeg 0" << std::endl;
    return -1;
  }

  // std::string server_address{"localhost:2510"};
  std::string server_address = argv[1];
  std::string img_file = argv[2];
  if (argc > 3) {
    if (std::string(argv[3]) == "1") {
      is_sync_mode = true;
    } else {
      is_sync_mode = false;
    }
  }
  
  std::cout << "Connecting to " << server_address << " and send img " << img_file
    << " with " << (is_sync_mode ? "sync" : "async") << " mode" << std::endl;
  SampleRequest capture;
  if (!processImage(img_file, "jpeg", capture)) {
    std::cout << "process image failed" << std::endl;
    return -1;
  }
  std::cout << "process image success" << std::endl;

  if (is_sync_mode) {
    SyncSampleClient client{grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials())};
    while (is_running_) {
        client.SampleMethod(capture);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } else {
    AsyncSampleClient client{grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials())};
    std::thread thread{&AsyncSampleClient::AsyncCompleteRPC, &client};
    while (is_running_) {
        client.SampleMethod(capture);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  return 0;
}