#include <mrs_errorgraph_viewer/errorgraph.h>

namespace mrs_errorgraph_viewer
{

  bool Errorgraph::has_loops()
  {
    // TODO
    return false;
  }

  std::vector<std::unique_ptr<Errorgraph::path_node_t>> Errorgraph::find_dependencies(const element_t& element, bool* loop_detected_out)
  {
    // std::vector<const path_node_t*> path_roots;
    std::vector<const element_t*> open_elements = find_previous_elements(element);
    std::vector<std::unique_ptr<path_node_t>> visited_elements;
    bool loop_detected = false;

    // basically DFS
    while (!open_elements.empty())
    {
      const auto cur_elem = open_elements.back();
      open_elements.pop_back();
      const std::vector<const element_t*> prevs = find_previous_elements(*cur_elem);
      for (const auto& el : prevs)
      {
        //TODO
      }
    }

    if (loop_detected_out != nullptr)
      *loop_detected_out = loop_detected;
    return visited_elements;
  }

  std::vector<const Errorgraph::element_t*> Errorgraph::find_all_roots()
  {
    std::vector<const element_t*> roots;
    for (const auto& el_ptr : elements_)
    {
      if (!el_ptr->is_waiting_for())
        roots.push_back(el_ptr.get());
    }
    return roots;
  }

  std::vector<const Errorgraph::element_t*> Errorgraph::find_previous_elements(const element_t& element)
  {
    std::vector<const element_t*> waiting_for_elements;

    const auto waiting_for_node_ids = element.waiting_for();
    for (const auto& node_id_ptr : waiting_for_node_ids)
    {
      const auto previous_el = find_element(*node_id_ptr);
      if (previous_el != nullptr)
        waiting_for_elements.push_back(previous_el);
    }
    return waiting_for_elements;
  }

  Errorgraph::element_t* Errorgraph::find_element_mutable(const node_id_t& node_id)
  {
    const auto elem_it = std::find_if(std::begin(elements_), std::end(elements_), [node_id](const auto& el_ptr)
      {
        return el_ptr->source_node == node_id;
      });
    if (elem_it == std::end(elements_))
      return nullptr;
    else
      return elem_it->get();
  }

  const Errorgraph::element_t* Errorgraph::find_element(const node_id_t& node_id)
  {
    return find_element_mutable(node_id);
  }

  const Errorgraph::element_t* Errorgraph::add_element_from_msg(const errorgraph_element_msg_t& msg)
  {
    node_id_t source_node_id = node_id_t::from_msg(msg.source_node);
    element_t* element = find_element_mutable(source_node_id);
    // if this element doesn't exist yet, construct it
    if (element == nullptr)
    {
      auto new_element_ptr = std::make_unique<element_t>(std::move(source_node_id));
      element = new_element_ptr.get();
      elements_.emplace_back(std::move(new_element_ptr));
    }

    element->errors.clear();
    for (const auto& error_msg : msg.errors)
      element->errors.emplace_back(error_msg);

    return element;
  }
}
