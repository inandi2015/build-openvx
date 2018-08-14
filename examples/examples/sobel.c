#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <VX/vx.h>
#include <VX/vx_helper.h>
#include <VX/vx_lib_debug.h>

int main(int argc, char *argv[]) {
  vx_status status = VX_SUCCESS;
  vx_uint32 width = 640;
  vx_uint32 height = 480;
  vx_uint32 th = 40;
  vx_uint32 i;

  vx_context context = vxCreateContext();
  if (context) {
    vxLoadKernels(context, "openvx-debug");

    vx_graph graph = vxCreateGraph(context);

    // Images
    vx_image images[] = {
        vxCreateImage(context, width, height, VX_DF_IMAGE_U8),
        vxCreateVirtualImage(graph, 0, 0, VX_DF_IMAGE_U8),
        vxCreateVirtualImage(graph, 0, 0, VX_DF_IMAGE_S16),
        vxCreateVirtualImage(graph, 0, 0, VX_DF_IMAGE_S16),
        vxCreateVirtualImage(graph, 0, 0, VX_DF_IMAGE_U8),
        vxCreateImage(context, width, height, VX_DF_IMAGE_U8),
    };

    vxuFReadImage(context, "../raw/bikegray_640x480.pgm", images[0]);

    // Constants
    vx_threshold thresh =
        vxCreateThreshold(context, VX_THRESHOLD_TYPE_BINARY, VX_TYPE_UINT8);
    vxSetThresholdAttribute(thresh, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_VALUE, &th,
                            sizeof(th));

    if (graph) {
      // Nodes
      vx_node nodes[] = {
          vxGaussian3x3Node(graph, images[0], images[1]),
          vxSobel3x3Node(graph, images[1], images[2], images[3]),
          vxMagnitudeNode(graph, images[2], images[3], images[4]),
          vxThresholdNode(graph, images[4], thresh, images[5]),
      };
      for (i = 0; i < dimof(nodes); i++) {
        if (nodes[i] == 0) {
          printf("Failed to make nodes[%u]\n", i);
        }
      }

      // GRAPH VERIFICATION
      status = vxVerifyGraph(graph);

      if (status != VX_SUCCESS)
        printf("VERIFICATION ERROR: %d\n", status);

      if (status == VX_SUCCESS) {
        // GRAPH EXECUTION

        status = vxProcessGraph(graph);

        // WRITE RESULT
        vxuFWriteImage(context, images[5], "./out.pgm");
      }
    }
  }
  return status;
}
