#include <mrs_errorgraph_viewer/errorgraph.h>

namespace mrs_errorgraph_viewer
{

  std::vector<const Errorgraph::element_t*> Errorgraph::find_dependency_roots(const node_id_t& node_id, bool* loop_detected_out)
  {
    // first, make sure that the elements are connected as the graph
    prepare_graph(elements_);
    const auto elem = find_element_mutable(node_id, elements_);
    return DFS(elem, elements_, loop_detected_out);
  }

  std::vector<const Errorgraph::element_t*> Errorgraph::DFS(element_t* from, const std::vector<std::unique_ptr<element_t>>& elements, bool* loop_detected_out)
  {
    std::vector<const element_t*> roots;
    std::vector<element_t*> open_elements;
    bool loop_detected = false;

    // prepare the first element into the open elements stack
    open_elements.push_back(from);

    // run the DFS
    while (!open_elements.empty())
    {
      auto cur_elem = open_elements.back();
      cur_elem->visited = true;
      open_elements.pop_back();
      const auto prevs = find_waiting_for(*cur_elem, elements);

      // if this element doesn't have any nodes it's waiting for, it is a root
      if (prevs.empty())
        roots.push_back(cur_elem);

      // if there are any nodes it's waiting for, process them
      for (auto& el : prevs)
      {
        // if the element was not visited, add it to the open list
        if (!el->visited)
        {
          open_elements.push_back(el);
        }
        // if this element was already visited, we have a loop
        else
        {
          // add the element as a root although it's within a loop
          // so that the dependency doesn't get lost in the output
          roots.push_back(el);
          loop_detected = true;
        }

        // create the both-sided connection
        el->parents.push_back(cur_elem);
        cur_elem->children.push_back(el);
      }
    }

    if (loop_detected_out != nullptr)
      *loop_detected_out = loop_detected;
    return roots;
  }

  void Errorgraph::build_graph(const std::vector<std::unique_ptr<element_t>>& elements)
  {
    // first, make sure that the elements are connected as the graph
    // and all helper member variables are reset
    prepare_graph(elements);
    auto last_unvisited = std::begin(elements);
    while (last_unvisited != std::end(elements))
    {
      const auto elem = last_unvisited->get();
      if (elem->visited)
      {
        last_unvisited++;
        continue;
      }
      DFS(elem, elements);
    }
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

  std::vector<const Errorgraph::element_t*> Errorgraph::find_all_leaves()
  {
    std::vector<const element_t*> leaves;
    for (const auto& el_ptr : elements_)
    {
      bool is_leaf = true;
      for (const auto& el_ptr2 : elements_)
      {
        if (el_ptr2->is_waiting_for(el_ptr->source_node))
        {
          is_leaf = false;
          break;
        }
      }
      if (is_leaf)
        leaves.push_back(el_ptr.get());
    }
    return leaves;
  }

  std::vector<Errorgraph::element_t*> Errorgraph::find_waiting_for(const element_t& element, const std::vector<std::unique_ptr<element_t>>& elements)
  {
    std::vector<element_t*> waiting_for_elements;

    const auto waiting_for_node_ids = element.waiting_for();
    for (const auto& node_id_ptr : waiting_for_node_ids)
    {
      const auto previous_el = find_element_mutable(*node_id_ptr, elements);
      if (previous_el != nullptr)
        waiting_for_elements.push_back(previous_el);
    }
    return waiting_for_elements;
  }

  Errorgraph::element_t* Errorgraph::find_element_mutable(const node_id_t& node_id, const std::vector<std::unique_ptr<element_t>>& elements)
  {
    const auto elem_it = std::find_if(std::begin(elements), std::end(elements), [node_id](const auto& el_ptr)
      {
        return el_ptr->source_node == node_id;
      });
    if (elem_it == std::end(elements))
      return nullptr;
    else
      return elem_it->get();
  }

  const Errorgraph::element_t* Errorgraph::find_element(const node_id_t& node_id)
  {
    return find_element_mutable(node_id, elements_);
  }

  void Errorgraph::prepare_graph(const std::vector<std::unique_ptr<element_t>>& elements)
  {
    for (const auto& el_ptr : elements)
    {
      el_ptr->children.clear();
      el_ptr->parents.clear();
      el_ptr->visited = false;
    }
  }

  const Errorgraph::element_t* Errorgraph::add_element_from_msg(const errorgraph_element_msg_t& msg)
  {
    node_id_t source_node_id = node_id_t::from_msg(msg.source_node);
    element_t* element = find_element_mutable(source_node_id, elements_);
    // if this element doesn't exist yet, construct it
    if (element == nullptr)
    {
      auto new_element_ptr = std::make_unique<element_t>(std::move(source_node_id), last_element_id++);
      element = new_element_ptr.get();
      elements_.emplace_back(std::move(new_element_ptr));
    }

    element->errors.clear();
    for (const auto& error_msg : msg.errors)
      element->errors.emplace_back(error_msg);

    return element;
  }

  void Errorgraph::write_dot()
  {
    build_graph(elements_);

    for (const auto& element : elements_)
    {
      std::cout << element->element_id << " [label=\"" << element->source_node << "\"];\n";
      for (const auto& child : element->children)
        std::cout << element->element_id << " -> " << child->element_id << ";\n";
    }
  }
}
