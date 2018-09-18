#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <VX/vx.h>
#include <VX/vx_helper.h>
#include <VX/vx_lib_debug.h>

int main(int argc, char *argv[]) {
  vx_status status = VX_SUCCESS;
  vx_uint32 width = 8000;   // 80;
  vx_uint32 height = 6000;  // 60;
  vx_uint32 th = 40;
  vx_uint32 i;

  vx_context context = vxCreateContext();
  if (context) {
    // vxLoadKernels(context, "openvx-c_model");
    vxLoadKernels(context, "openvx-debug");
    vxLoadKernels(context, "openvx-extras");

    // Graph init
    vx_graph graph = vxCreateGraph(context);

    vx_image images[] = {
        vxCreateImage(context, width, height, VX_DF_IMAGE_U8),
    };

    vx_float32 _strength_thresh = 10;
    vx_float32 _min_distance = 4.0;
    vx_float32 _sensitivity = 0.1;
    vx_scalar strength_thresh =
        vxCreateScalar(context, VX_TYPE_FLOAT32, &_strength_thresh);
    vx_scalar min_distance =
        vxCreateScalar(context, VX_TYPE_FLOAT32, &_min_distance);
    vx_scalar sensitivity =
        vxCreateScalar(context, VX_TYPE_FLOAT32, &_sensitivity);
    vx_array corners = vxCreateArray(context, VX_TYPE_KEYPOINT, 640);
    vx_scalar num_corners = vxCreateScalar(context, VX_TYPE_SIZE, NULL);
    vx_int32 gradient_size = 3;
    vx_int32 block_size = 3;

    if (graph) {
      // Nodes
      vx_node nodes[] = {
          vxHarrisCornersNode(graph, images[0], strength_thresh, min_distance,
                              sensitivity, gradient_size, block_size, corners,
                              num_corners),
      };

      for (i = 0; i < dimof(nodes); i++)
        if (nodes[i] == 0) printf("Failed to make nodes[%u]\n", i);

      // GRAPH VERIFICATION
      status = vxVerifyGraph(graph);

      printf("AFTER VERIFICATION (status: %d)\n", status);

      if (status == VX_SUCCESS) {
        // GRAPH EXECUTION
        status = vxProcessGraph(graph);
        printf("SUCCESS\n");
      }
      // vxReleaseGraph(&graph); /* node destruction is implicit */
    }
  }
  // vxReleaseContext(&context);
  return status;
}
