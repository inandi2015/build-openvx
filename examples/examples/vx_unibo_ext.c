
#include <VX/vx.h>
#include <VX/vx_helper.h>
#include "vx_ext_unibo.h"

////////////////////////////////////////////////////////////////////////////////
/// ComplementaryKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxComplementaryNode(vx_graph graph, vx_image input, vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel =
        vxGetKernelByName(context, "it.unibo.retina.complementary");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[2];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuComplementary(vx_image input, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxComplementaryNode(graph, input, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// NormalizeKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxNormalizeNode(vx_graph graph, vx_image input, vx_scalar min,
                        vx_scalar max, vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;  // vxLoadKernels(context, "retina");
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.normalize");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[4];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)min);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)max);
        statuses[3] = vxSetParameterByIndex(node, 3, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuNormalize(vx_image input, vx_scalar min, vx_scalar max,
                       vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxNormalizeNode(graph, input, min, max, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// GaussianKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxGaussian5x5Node(vx_graph graph, vx_image input, vx_scalar sigma,
                          vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel =
        vxGetKernelByName(context, "it.unibo.retina.gaussian5x5");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)sigma);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuGaussian5x5(vx_image input, vx_scalar sigma, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxGaussian5x5Node(graph, input, sigma, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// ThresholdToZeroKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxThresholdToZeroNode(vx_graph graph, vx_image input, vx_scalar th,
                              vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel =
        vxGetKernelByName(context, "it.unibo.retina.threshold_to_zero");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)th);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuThresholdToZero(vx_image input, vx_scalar th, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxThresholdToZeroNode(graph, input, th, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

vx_node vxTruncateNode(vx_graph graph, vx_image input, vx_scalar th,
                       vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.truncate");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)th);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuTruncateToZero(vx_image input, vx_scalar th, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxTruncateNode(graph, input, th, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

vx_node vxAddScalarNode(vx_graph graph, vx_image input, vx_scalar value,
                        vx_scalar scale, vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.add_scalar");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[4];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)value);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)scale);
        statuses[3] = vxSetParameterByIndex(node, 3, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuAddScalar(vx_image input, vx_scalar value, vx_scalar scale,
                       vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxAddScalarNode(graph, input, value, scale, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

vx_node vxDivideNode(vx_graph graph, vx_image input1, vx_image input2,
                     vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.divide");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input1);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)input2);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuDivide(vx_image input1, vx_image input2, vx_image output) {
  vx_context context = vxGetContext(input1);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxDivideNode(graph, input1, input2, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

vx_node vxSigmoidNode(vx_graph graph, vx_image input, vx_scalar mean,
                      vx_scalar std, vx_scalar slope, vx_scalar minRange,
                      vx_scalar maxRange, vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.sigmoid");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[7];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)mean);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)std);
        statuses[3] = vxSetParameterByIndex(node, 3, (vx_reference)slope);
        statuses[4] = vxSetParameterByIndex(node, 4, (vx_reference)minRange);
        statuses[5] = vxSetParameterByIndex(node, 5, (vx_reference)maxRange);
        statuses[6] = vxSetParameterByIndex(node, 6, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuSigmoid(vx_image input, vx_scalar mean, vx_scalar std,
                     vx_scalar slope, vx_scalar minRange, vx_scalar maxRange,
                     vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxSigmoidNode(graph, input, mean, std, slope, minRange,
                                 maxRange, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// MeanKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxMeanNode(vx_graph graph, vx_image input, vx_scalar mean) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.mean");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[2];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)mean);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuMean(vx_image input, vx_scalar mean) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxMeanNode(graph, input, mean);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// StdDevKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxStdDevNode(vx_graph graph, vx_image input, vx_scalar mean,
                     vx_scalar std) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.stddev");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)mean);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)std);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuStdDev(vx_image input, vx_scalar mean, vx_scalar std) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxStdDevNode(graph, input, mean, std);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// Max3x3Kernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxMax3x3Node(vx_graph graph, vx_image input, vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.max3x3");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[2];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuMax3x3(vx_image input, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxMax3x3Node(graph, input, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// Max7x7Kernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxMax7x7Node(vx_graph graph, vx_image input, vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.max7x7");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[2];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuMax7x7(vx_image input, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxMax7x7Node(graph, input, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// NonMaximaSuppressionKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxNonMaxSuppression2Node(vx_graph graph, vx_image input, vx_image max,
                                 vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.nonmax");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)max);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuNonMax2Suppression(vx_image input, vx_image max, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxNonMaxSuppression2Node(graph, input, max, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// FAST9Kernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxFAST9Node(vx_graph graph, vx_image input, vx_scalar th,
                    vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.fast9");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)th);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuFAST9(vx_image input, vx_scalar th, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxFAST9Node(graph, input, th, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// IntegralWindowKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxIntegralWindowNode(vx_graph graph, vx_image input, vx_scalar winSize,
                             vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel =
        vxGetKernelByName(context, "it.unibo.retina.integral_window");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[3];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)winSize);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuIntegralWindow(vx_image input, vx_scalar winSize,
                            vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node = vxIntegralWindowNode(graph, input, winSize, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// DisparityKernel
////////////////////////////////////////////////////////////////////////////////

vx_node vxDisparityNode(vx_graph graph, vx_image input, vx_image minVals,
                        vx_image current, vx_scalar level, vx_image minOut,
                        vx_image output) {
  vx_uint32 i;
  vx_node node = 0;
  vx_context context = vxGetContext(graph);
  vx_status status = VX_SUCCESS;
  if (status == VX_SUCCESS) {
    vx_kernel kernel = vxGetKernelByName(context, "it.unibo.retina.disparity");
    if (kernel) {
      node = vxCreateGenericNode(graph, kernel);
      if (node) {
        vx_status statuses[6];
        statuses[0] = vxSetParameterByIndex(node, 0, (vx_reference)input);
        statuses[1] = vxSetParameterByIndex(node, 1, (vx_reference)minVals);
        statuses[2] = vxSetParameterByIndex(node, 2, (vx_reference)current);
        statuses[3] = vxSetParameterByIndex(node, 3, (vx_reference)level);
        statuses[4] = vxSetParameterByIndex(node, 4, (vx_reference)minOut);
        statuses[5] = vxSetParameterByIndex(node, 5, (vx_reference)output);
        for (i = 0; i < dimof(statuses); i++) {
          if (statuses[i] != VX_SUCCESS) {
            status = VX_ERROR_INVALID_PARAMETERS;
            vxReleaseNode(&node);
            vxReleaseKernel(&kernel);
            node = 0;
            kernel = 0;
            break;
          }
        }
      } else {
        vxReleaseKernel(&kernel);
      }
    }
  }
  return node;
}

vx_status vxuDisparity(vx_image input, vx_image minVals, vx_image current,
                       vx_scalar level, vx_image minOut, vx_image output) {
  vx_context context = vxGetContext(input);
  vx_status status = VX_FAILURE;
  vx_graph graph = vxCreateGraph(context);
  if (graph) {
    vx_node node =
        vxDisparityNode(graph, input, minVals, current, level, minOut, output);
    if (node) {
      status = vxVerifyGraph(graph);
      if (status == VX_SUCCESS) {
        status = vxProcessGraph(graph);
      }
      vxReleaseNode(&node);
    }
    vxReleaseGraph(&graph);
  }
  return status;
}
