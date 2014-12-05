float32 x
float32 y
uint32 ref # can't have a list of nodes in a node, so use this to refer to nodes
bool object # is this node the (approximate) position of an object
uint32[] links # list of refs to other nodes which this one is linked to