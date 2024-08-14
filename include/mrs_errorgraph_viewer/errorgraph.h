#pragma once

#include <string>
#include <optional>
#include <vector>
#include <memory>

#include <mrs_robot_diagnostics/ErrorgraphNodeID.h>
#include <mrs_robot_diagnostics/ErrorgraphError.h>

namespace mrs_errorgraph_viewer
{

  struct node_id_t
  {
    std::string node;
    std::string component;

    bool operator==(const node_id_t& other) const
    {
      return node == other.node;
    }
  };

  struct errorgraph_error_t
  {
    std::string type;
    std::optional<node_id_t> waiting_for_node;

    inline bool is_waiting_for() const
    {
      return type == mrs_robot_diagnostics::ErrorgraphError::ERROR_TYPE_WAITING_FOR;
    }

    inline bool is_no_error() const
    {
      return type == mrs_robot_diagnostics::ErrorgraphError::ERROR_TYPE_NO_ERROR;
    }
  };

  struct errorgraph_element_t
  {
    node_id_t source_node;
    std::vector<errorgraph_error_t> errors;

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

  class Errorgraph
  {
    private:
      std::vector<std::unique_ptr<errorgraph_element_t>> elements_;

      std::vector<const errorgraph_element_t*> find_previous_elements(const errorgraph_element_t& element);

    public:
      bool has_loops();
      std::vector<const errorgraph_element_t*> find_roots(const errorgraph_element_t& element, bool* loop_detected_out);
  };

}
