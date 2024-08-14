#include <mrs_errorgraph_viewer/errorgraph.h>

namespace mrs_errorgraph_viewer
{

  bool Errorgraph::has_loops()
  {
    bool loop_detected = false;
    for (const auto& el : elements_)
    {
      find_roots(*el, &loop_detected);
      if (loop_detected)
        return true;
    }
    return false;
  }

  std::vector<const errorgraph_element_t*> Errorgraph::find_roots(const errorgraph_element_t& element, bool* loop_detected_out = nullptr)
  {
    std::vector<const errorgraph_element_t*> roots;
    std::vector<const errorgraph_element_t*> open_elements = find_previous_elements(element);
    std::vector<const errorgraph_element_t*> visited_elements = open_elements; // copy happening here!
    bool loop_detected = false;

    // basically DFS
    while (!open_elements.empty())
    {
      const auto cur_elem = open_elements.back();
      open_elements.pop_back();
      const std::vector<const errorgraph_element_t*> prevs = find_previous_elements(*cur_elem);
      for (const auto& el : prevs)
      {
        const bool already_visited = std::any_of(std::begin(visited_elements), std::end(visited_elements), [el](const auto& visited_el)
            {
              return el == visited_el;
            });
        if (already_visited)
        {
          loop_detected = true;
          roots.push_back(cur_elem);
        }
        else
          open_elements.push_back(el);
      }
      // if there are no more previous elements, this is one of the roots
      if (prevs.empty())
        roots.push_back(cur_elem);
    }

    if (loop_detected_out != nullptr)
      *loop_detected_out = loop_detected;
    return roots;
  }

  std::vector<const errorgraph_element_t*> Errorgraph::find_previous_elements(const errorgraph_element_t& element)
  {
    std::vector<const errorgraph_element_t*> waiting_for_elements;

    const auto waiting_for_node_ids = element.waiting_for();
    for (const auto& node_id_ptr : waiting_for_node_ids)
    {
      const auto previous_el = find_element(*node_id_ptr);
      if (previous_el != nullptr)
        waiting_for_elements.push_back(previous_el);
    }
    return waiting_for_elements;
  }

  errorgraph_element_t* Errorgraph::find_element_mutable(const node_id_t& node_id)
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

  const errorgraph_element_t* Errorgraph::find_element(const node_id_t& node_id)
  {
    return find_element_mutable(node_id);
  }

  const errorgraph_element_t* Errorgraph::add_element_from_msg(const errorgraph_element_msg_t& msg)
  {
    node_id_t source_node_id = node_id_t::from_msg(msg.source_node);
    errorgraph_element_t* element = find_element_mutable(source_node_id);
    // if this element doesn't exist yet, construct it
    if (element == nullptr)
    {
      auto new_element_ptr = std::make_unique<errorgraph_element_t>(std::move(source_node_id));
      element = new_element_ptr.get();
      elements_.emplace_back(std::move(new_element_ptr));
    }

    element->errors.clear();
    for (const auto& error_msg : msg.errors)
      element->errors.emplace_back(error_msg);

    return element;
  }
}
