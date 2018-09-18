#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <VX/vx.h>
#include <VX/vx_helper.h>
#include <VX/vx_lib_debug.h>

int main(int argc, char *argv[]) {
  vx_status status = VX_SUCCESS;
  vx_uint32 width = 6400;   // 640
  vx_uint32 height = 4800;  // 480
  vx_uint32 i;

  vx_context context = vxCreateContext();
  if (context) {
    // vxLoadKernels(context, "openvx-c_model");
    vxLoadKernels(context, "openvx-debug");

    // Graph init
    vx_graph graph = vxCreateGraph(context);

    // Images
    vx_image images[] = {
        vxCreateImage(context, width, height, VX_DF_IMAGE_U8),
        vxCreateImage(context, width, height, VX_DF_IMAGE_U8),
    };

    vxuFReadImage(context, "../raw/bikegray_6400x4800.pgm", images[0]);

    // Constants
    vx_uint32 lo = 6, hi = 10;
    vx_threshold hyst =
        vxCreateThreshold(context, VX_THRESHOLD_TYPE_RANGE, VX_TYPE_UINT8);
    vxSetThresholdAttribute(hyst, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_LOWER, &lo,
                            sizeof(lo));
    vxSetThresholdAttribute(hyst, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_UPPER, &hi,
                            sizeof(hi));
    vx_int32 winsz = 3;

    if (graph) {
      // Nodes
      vx_node nodes[] = {
          vxCannyEdgeDetectorNode(graph, images[0], hyst, winsz, VX_NORM_L2,
                                  images[1]),
      };
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

        vxuFWriteImage(context, images[1], "./out.pgm");
      }
    }
  }
  return status;
}
