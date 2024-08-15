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

    inline bool is_waiting_for(const node_id_t& node_id) const
    {
      return type == errorgraph_error_msg_t::ERROR_TYPE_WAITING_FOR && waiting_for_node.has_value() && waiting_for_node.value() == node_id;
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

      struct path_node_t;

      struct element_t
      {
        node_id_t source_node;
        std::vector<errorgraph_error_t> errors;

        // graph-related variables
        std::vector<element_t*> parents;
        std::vector<element_t*> children;
        bool visited;

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

        inline bool is_waiting_for(const node_id_t& node_id) const
        {
          return std::any_of(std::begin(errors), std::end(errors), [node_id](const auto& error)
              {
                return error.is_waiting_for(node_id);
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

    private:
      std::vector<std::unique_ptr<element_t>> elements_;

      static std::vector<element_t*> find_waiting_for(const element_t& element, const std::vector<std::unique_ptr<element_t>>& elements);

      static element_t* find_element_mutable(const node_id_t& node_id, const std::vector<std::unique_ptr<element_t>>& elements);

      static void prepare_graph(const std::vector<std::unique_ptr<element_t>>& elements);

      static void build_graph(const std::vector<std::unique_ptr<element_t>>& elements);

      static std::vector<const element_t*> DFS(element_t* from, const std::vector<std::unique_ptr<element_t>>& elements, bool* loop_detected_out = nullptr);

    public:

      std::vector<const element_t* > find_dependency_roots(const node_id_t& node_id, bool* loop_detected_out = nullptr);

      std::vector<const element_t*> find_all_roots();

      std::vector<const element_t*> find_all_leaves();

      const element_t* find_element(const node_id_t& node_id);

      const element_t* add_element_from_msg(const errorgraph_element_msg_t& msg);
  };

}
