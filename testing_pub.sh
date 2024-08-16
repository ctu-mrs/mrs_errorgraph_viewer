
rostopic pub -1 /uav1/errors mrs_errorgraph_viewer/ErrorgraphElement "stamp:
    secs: 0
    nsecs: 0
source_node:
  node: 'root1'
  component: 'comp1'
errors:
- error_type: 'waiting for hardware or something'
  waiting_for_node:
    node: ''
    component: ''
- error_type: 'something borken'
  waiting_for_node:
    node: ''
    component: ''"

rostopic pub -1 /uav1/errors mrs_errorgraph_viewer/ErrorgraphElement "stamp:
    secs: 0
    nsecs: 0
source_node:
  node: 'leaf1'
  component: 'comp5'
errors:
- error_type: 'waiting_for'
  waiting_for_node:
    node: 'node1'
    component: 'comp1'"

rostopic pub -1 /uav1/errors mrs_errorgraph_viewer/ErrorgraphElement "stamp:
    secs: 0
    nsecs: 0
source_node:
  node: 'node1'
  component: 'comp1'
errors:
- error_type: 'waiting_for'
  waiting_for_node:
    node: 'node2'
    component: 'comp1'
- error_type: 'icecream machine broke'
  waiting_for_node:
    node: ''
    component: ''
- error_type: 'waiting_for'
  waiting_for_node:
    node: 'root1'
    component: 'comp1'"

rostopic pub -1 /uav1/errors mrs_errorgraph_viewer/ErrorgraphElement "stamp:
    secs: 0
    nsecs: 0
source_node:
  node: 'node2'
  component: 'comp1'
errors:
- error_type: 'waiting_for'
  waiting_for_node:
    node: 'node1'
    component: 'comp1'"

rostopic pub -1 /uav1/errors mrs_errorgraph_viewer/ErrorgraphElement "stamp:
    secs: 0
    nsecs: 0
source_node:
  node: 'leaf2'
  component: 'comp1'
errors:
- error_type: 'waiting_for'
  waiting_for_node:
    node: 'root1'
    component: 'comp1'"
