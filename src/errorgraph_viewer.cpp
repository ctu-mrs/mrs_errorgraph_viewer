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

      // const std::string ofname = "/tmp/errorgraph.dot";
      // std::ofstream ofs(ofname);
      // if (!ofs.is_open())
      // {
      //   ROS_ERROR_STREAM("[ErrorgraphViewer]: Could not open file \"" << ofname << "\" for writing. Not showing graph vizualization.");
      //   return;
      // }
      // errorgraph_.write_dot(ofs);

      std::stringstream ss;
      errorgraph_.write_dot(ss);
      const std::string graph_dot = ss.str();
      GVC_t *gvc = gvContext();
      graph_t *g = agmemread(graph_dot.c_str());

      gvLayout(gvc, g, "dot");
      char *raw_png_data;
      unsigned int raw_png_data_len;
      gvRenderData(gvc, g, "png", &raw_png_data, &raw_png_data_len);
      gvFreeLayout(gvc, g);
      agclose(g);
      gvFreeContext(gvc);

      std::vector<char> vec_png_data;
      vec_png_data.insert(std::end(vec_png_data), raw_png_data, raw_png_data + raw_png_data_len);
      gvFreeRenderData(raw_png_data);

      cv::Mat image = cv::imdecode(std::move(vec_png_data), cv::ImreadModes::IMREAD_COLOR);
      cv::imshow("Errorgraph visualization", image);
      cv::waitKey(10);

      // /* rendering using system + imread + imshow //{ */
      
      // const std::string img_ofname = "/tmp/errorgraph.png";
      // const auto sys_cmd = "dot -Tpng -o " + img_ofname + " " + ofname;
      // const auto ret = system(sys_cmd.c_str());
      // ROS_INFO_STREAM("[ErrorgraphViewer]: Executing command \"" << sys_cmd << "\" to produce graph image.");
      // if (ret != 0)
      // {
      //   ROS_ERROR_STREAM("[ErrorgraphViewer]: Command \"" << sys_cmd << "\" returned error code " << ret << ". Not showing graph visualization.");
      //   return;
      // }
      // // ROS_INFO_STREAM("[ErrorgraphViewer]: Command \"" << sys_cmd << "\" returned error code " << ret << ". Not showing graph visualization.");
      
      // const cv::Mat image = cv::imread(img_ofname);
      // if (image.empty())
      // {
      //   ROS_ERROR_STREAM("[ErrorgraphViewer]: Failed to open image file \"" << img_ofname << "\". Not showing graph visualization.");
      //   return;
      // }
      
      // cv::imshow("Errorgraph visualization", image);
      // cv::waitKey(10);
      
      //}

      // const auto leaves = errorgraph_.find_all_leaves();

//       std::cout << "Error dependencies are:\n";
//       for (const auto& leaf : leaves)
//       {
//         const auto roots = errorgraph_.find_dependency_roots(leaf->source_node);
//         std::vector<element_with_depth_t> open_elements;
//         open_elements.reserve(roots.size());
//         for (const auto& el_ptr : roots)
//           open_elements.emplace_back(el_ptr, 0);

//         while (!open_elements.empty())
//         {
//           auto cur_elem = open_elements.back();
//           open_elements.pop_back();
//           for (int it = 0; it < cur_elem.depth; it++)
//             std::cout << "\t";
//           if (cur_elem.depth > 0)
//             std::cout << "└ ";
//           else
//             std::cout << "x ";
//           std::cout << cur_elem.element->source_node << ":\t";
//           for (int it = 0; it < cur_elem.element->errors.size(); it++)
//           {
//             std::cout << cur_elem.element->errors.at(it).type;
//             if (it+1 < cur_elem.element->errors.size())
//               std::cout << ",";
//           }
//           std::cout << "\n";


//           open_elements.reserve(open_elements.size() + cur_elem.element->parents.size());
//           for (const auto& el_ptr : cur_elem.element->parents)
//             open_elements.emplace_back(el_ptr, cur_elem.depth+1);
//         }
//       }
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
