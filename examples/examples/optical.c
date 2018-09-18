#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <VX/vx.h>
#include <VX/vx_helper.h>
#include <VX/vx_lib_debug.h>

int main(int argc, char *argv[]) {
  vx_status status = VX_SUCCESS;
  vx_uint32 width = 3200;   // 320;
  vx_uint32 height = 2400;  // 240;
  vx_uint32 th = 40;
  vx_uint32 i;

  vx_context context = vxCreateContext();
  if (context) {
    vxLoadKernels(context, "openvx-debug");

    // Graph init
    vx_graph graph = vxCreateGraph(context);

    vx_uint32 winSize = 320;  // 32
    vx_float32 epsilon = 0.01;
    vx_uint32 num_iterations = 100;  // 10
    vx_bool use_initial_estimate = vx_true_e;
    vx_enum criteria = VX_TERM_CRITERIA_BOTH;  // lk params

    vx_scalar epsilon_s = vxCreateScalar(context, VX_TYPE_FLOAT32, &epsilon);
    vx_scalar num_iterations_s =
        vxCreateScalar(context, VX_TYPE_UINT32, &num_iterations);
    vx_scalar use_initial_estimate_s =
        vxCreateScalar(context, VX_TYPE_BOOL, &use_initial_estimate);

    vx_pyramid pyramid_new = vxCreatePyramid(context, 4, VX_SCALE_PYRAMID_HALF,
                                             width, height, VX_DF_IMAGE_U8);
    vx_pyramid pyramid_old = vxCreatePyramid(context, 4, VX_SCALE_PYRAMID_HALF,
                                             width, height, VX_DF_IMAGE_U8);
    vx_array old_features = vxCreateArray(context, VX_TYPE_KEYPOINT, 1000);
    vx_array new_features = vxCreateArray(context, VX_TYPE_KEYPOINT, 1000);

    if (graph) {
      // Nodes
      vx_node nodes[] = {vxOpticalFlowPyrLKNode(
          graph, pyramid_old, pyramid_new, old_features, old_features,
          new_features, criteria, epsilon_s, num_iterations_s,
          use_initial_estimate_s, winSize)};
      for (i = 0; i < dimof(nodes); i++) {
        if (nodes[i] == 0) {
          printf("Failed to make nodes[%u]\n", i);
        }
      }

      // GRAPH VERIFICATION
      status = vxVerifyGraph(graph);

      printf("AFTER VERIFICATION (status: %d)\n", status);

      if (status == VX_SUCCESS) {
        // GRAPH EXECUTION

        status = vxProcessGraph(graph);
      }
    }
  }
  return status;
}
