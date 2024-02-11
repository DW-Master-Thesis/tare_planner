# Keypose Graph
# In TARE, graph and dist are 2D arrays
# Msg uses a 1D array with a offset variable
# graph_in_tare[i][j] = i * graph_array_offset + j
# graph[i*graph_array_offset + j] = graph_in_tare[i][j]
# where graph_array_offset = sum of graph_array_offsets up to i
# ie
#   int graph_array_offset = 0
#   for (k = 0; k < i; k++) {
#    graph_array_offset += graph_array_offsets[k]}

bool allow_vertical_edge
int32 current_keypose_id
geometry_msgs/Point current_keypose_position
int32[] graph
int32[] graph_array_offsets
float64[] dist
int32[] dist_array_offsets
KeyposeNode[] nodes
int32[] connected_node_indices
