#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <VX/vx.h>
#include <VX/vx_helper.h>
#include <VX/vx_lib_debug.h>

int main(int argc, char *argv[]) {
  vx_uint32 i;
  vx_status status = VX_SUCCESS;
  vx_uint32 width = 640;
  vx_uint32 height = 480;

  vx_context context = vxCreateContext();
  if (context) {
    vxLoadKernels(context, "openvx-debug");

    // Graph init
    vx_graph graph = vxCreateGraph(context);

    // Input image
    vx_image input = vxCreateImage(context, width, height, VX_DF_IMAGE_U8);

    // Read image from file
    vxuFReadImage(context, "../raw/bikegray_640x480.pgm", input);

    // Pyramid data structure
    vx_pyramid pyramid = vxCreatePyramid(context, 4, VX_SCALE_PYRAMID_HALF,
                                         width, height, VX_DF_IMAGE_U8);

    if (graph) {
      // Nodes
      vx_node nodes[] = {
          vxGaussianPyramidNode(graph, input, pyramid),
      };
      for (i = 0; i < dimof(nodes); i++) {
        if (nodes[i] == 0) {
          printf("Failed to make nodes[%u]\n", i);
        }
      }

      // GRAPH VERIFICATION
      status = vxVerifyGraph(graph);

      if (status == VX_SUCCESS) {
        // GRAPH EXECUTION

        status = vxProcessGraph(graph);

        vxuFWriteImage(context, vxGetPyramidLevel(pyramid, 1), "./out.pgm");
      }
    }
  }
  // vxReleaseContext(&context);
  return status;
}
