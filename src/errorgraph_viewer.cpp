/* includes //{ */

#include <fstream>

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>    

#include <graphviz/gvc.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_errorgraph_viewer/errorgraph.h>

//}

namespace mrs_errorgraph_viewer
{

  /* class ErrorgraphViewer //{ */

  class ErrorgraphViewer : public nodelet::Nodelet
  {
  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;

    std::mutex errorgraph_mtx_;
    Errorgraph errorgraph_;

    // mini-helper struct to properly release the Graphviz context
    struct GVC_deleter_t
    {
      void operator()(GVC_t* p)
      {
        gvFreeContext(p);
      }
    };
    std::unique_ptr<GVC_t, GVC_deleter_t> graphviz_context_; 

    // | ---------------------- ROS subscribers --------------------- |

    std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

    mrs_lib::SubscribeHandler<errorgraph_element_msg_t> sh_errorgraph_error_msg_;

    // | ----------------------- main timer ----------------------- |

    struct element_with_depth_t
    {
      const Errorgraph::element_t* element;
      int depth;
      element_with_depth_t(const Errorgraph::element_t* const element, const int depth)
        : element(element), depth(depth) {};
    };

    ros::Timer timer_main_;
    void timerMain(const ros::TimerEvent& event)
    {
      std::scoped_lock lck(errorgraph_mtx_);

      // Write the current errorgraph to a string in the dot format
      std::stringstream ss;
      errorgraph_.write_dot(ss);
      const std::string graph_dot = ss.str();
      // load the errorgraph to Graphviz from the string
      graph_t *g = agmemread(graph_dot.c_str());

      // use Graphviz to generate a PNG image of the graph
      gvLayout(graphviz_context_.get(), g, "dot");
      char *raw_png_data;
      unsigned int raw_png_data_len;
      gvRenderData(graphviz_context_.get(), g, "png", &raw_png_data, &raw_png_data_len);
      gvFreeLayout(graphviz_context_.get(), g);
      agclose(g);

      // copy the PNG data from a C-style array to a std::vector and free the C array TODO: avoid the copy here somehow
      std::vector<char> vec_png_data;
      vec_png_data.insert(std::end(vec_png_data), raw_png_data, raw_png_data + raw_png_data_len);
      gvFreeRenderData(raw_png_data);

      // display the image using OpenCV!
      cv::Mat image = cv::imdecode(std::move(vec_png_data), cv::ImreadModes::IMREAD_COLOR);
      cv::imshow("Errorgraph visualization", image);
      cv::waitKey(10);
    }

    // | ----------------- error message callback ----------------- |
    void cbkElement(const errorgraph_element_msg_t::ConstPtr element_msg)
    {
      std::scoped_lock lck(errorgraph_mtx_);
      errorgraph_.add_element_from_msg(*element_msg);
    }

    // | ------------------ Additional functions ------------------ |

  };
  //}

  /* onInit() //{ */

  void ErrorgraphViewer::onInit()
  {

    /* obtain node handle */
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    /* load parameters */
    mrs_lib::ParamLoader param_loader(nh_, "ErrorgraphViewer");

    std::string custom_config_path;

    param_loader.loadParam("custom_config", custom_config_path);

    if (custom_config_path != "")
    {
      param_loader.addYamlFile(custom_config_path);
    }

    param_loader.addYamlFileFromParam("config");

    const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[ErrorgraphViewer]: Could not load all parameters!");
      ros::shutdown();
    }

    graphviz_context_ = std::unique_ptr<GVC_t, GVC_deleter_t>(gvContext());

    // | ----------------------- subscribers ---------------------- |

    tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh_;
    shopts.node_name = "ErrorgraphViewer";
    shopts.no_message_timeout = ros::Duration(5.0);
    shopts.timeout_manager = tim_mgr_;
    shopts.threadsafe = true;
    shopts.autostart = true;
    shopts.queue_size = 10;
    shopts.transport_hints = ros::TransportHints().tcpNoDelay();

    // | --------------------- Main subscriber -------------------- |
    sh_errorgraph_error_msg_ = mrs_lib::SubscribeHandler<errorgraph_element_msg_t>(shopts, "in/errors", &ErrorgraphViewer::cbkElement, this);

    // | ------------------------- timers ------------------------- |

    timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &ErrorgraphViewer::timerMain, this);

    // | --------------------- finish the init -------------------- |

    ROS_INFO("[ErrorgraphViewer]: initialized");
    ROS_INFO("[ErrorgraphViewer]: --------------------");
  }

  //}

}  // namespace mrs_errorgraph_viewer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_errorgraph_viewer::ErrorgraphViewer, nodelet::Nodelet);
