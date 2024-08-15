#pragma once

#include <string>
#include <optional>
#include <vector>
#include <memory>
#include <algorithm>

#include <ros/time.h>

#include <mrs_errorgraph_viewer/ErrorgraphNodeID.h>
#include <mrs_errorgraph_viewer/ErrorgraphElement.h>

namespace mrs_errorgraph_viewer
{
  using errorgraph_node_id_msg_t = mrs_errorgraph_viewer::ErrorgraphNodeID;
  using errorgraph_error_msg_t = mrs_errorgraph_viewer::ErrorgraphError;
  using errorgraph_element_msg_t = mrs_errorgraph_viewer::ErrorgraphElement;

  struct node_id_t
  {
    std::string node;
    std::string component;

    inline bool operator==(const node_id_t& other) const
    {
      return node == other.node && component == other.component;
    }

    static inline node_id_t from_msg(const errorgraph_node_id_msg_t& msg)
    {
      node_id_t ret;
      ret.node = msg.node;
      ret.component = msg.component;
      return ret;
    }
  };

  inline std::ostream& operator<<(std::ostream& os, const node_id_t& node_id)
  {
    return os << node_id.node << "." << node_id.component;
  }

  struct errorgraph_error_t
  {
    ros::Time stamp;
    std::string type;
    std::optional<node_id_t> waiting_for_node;

    inline bool is_waiting_for() const
    {
      return type == errorgraph_error_msg_t::ERROR_TYPE_WAITING_FOR;
    }

    inline bool is_no_error() const
    {
      return type == errorgraph_error_msg_t::ERROR_TYPE_NO_ERROR;
    }

    errorgraph_error_t(const errorgraph_error_msg_t& msg)
      : stamp(msg.stamp), type(msg.error_type)
    {
      if (msg.error_type == errorgraph_error_msg_t::ERROR_TYPE_WAITING_FOR)
        waiting_for_node = node_id_t::from_msg(msg.waiting_for_node);
    }
  };

  class Errorgraph
  {
    public:

      struct element_t
      {
        node_id_t source_node;
        std::vector<errorgraph_error_t> errors;

        element_t(node_id_t&& source_node)
          : source_node(source_node) {};

        inline std::vector<const node_id_t*> waiting_for() const
        {
          std::vector<const node_id_t*> ret;
          ret.reserve(errors.size());
          for (const auto& el : errors)
          {
            if (el.waiting_for_node.has_value())
              ret.push_back(&el.waiting_for_node.value());
          }
          return ret;
        }

        inline bool is_waiting_for() const
        {
          return std::any_of(std::begin(errors), std::end(errors), [](const auto& error)
              {
                return error.is_waiting_for();
              });
        }

        inline bool is_no_error() const
        {
          return std::all_of(std::begin(errors), std::end(errors), [](const auto& error)
              {
                return error.is_no_error();
              });
        }
      };

      struct path_node_t
      {
        std::vector<path_node_t*> parents;
        std::vector<path_node_t*> children;
        element_t* element;
      };

    private:
      std::vector<std::unique_ptr<element_t>> elements_;

      std::vector<const element_t*> find_previous_elements(const element_t& element);

      element_t* find_element_mutable(const node_id_t& node_id);

    public:

      bool has_loops();

      std::vector<std::unique_ptr<path_node_t>> find_dependencies(const element_t& element, bool* loop_detected_out);

      std::vector<const element_t*> find_all_roots();

      const element_t* find_element(const node_id_t& node_id);

      const element_t* add_element_from_msg(const errorgraph_element_msg_t& msg);
  };

}
