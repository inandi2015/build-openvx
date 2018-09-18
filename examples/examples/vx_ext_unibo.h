#pragma once
#ifndef _OPENVX_EXT_RETINA_H_
#define _OPENVX_EXT_RETINA_H_

#include <VX/vx.h>

//! [KERNEL ENUM]
/*! \brief The Retina Library Set
 */
#define VX_LIBRARY_RETINA (0x2)

/*! \brief The list of Retina Kernels.
 */
enum vx_kernel_retina_ext_e {
  VX_KERNEL_UNIBO_COMPLEMENTARY =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x0,
  VX_KERNEL_UNIBO_NORMALIZE =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x1,
  VX_KERNEL_UNIBO_GAUSSIAN5X5 =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x2,
  VX_KERNEL_UNIBO_THRESHOLD_TO_ZERO =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x3,
  VX_KERNEL_UNIBO_TRUNCATE =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x4,
  VX_KERNEL_UNIBO_ADD_SCALAR =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x5,
  VX_KERNEL_UNIBO_DIVIDE =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x6,
  VX_KERNEL_UNIBO_SIGMOID =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x7,
  VX_KERNEL_UNIBO_MEAN = VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x8,
  VX_KERNEL_UNIBO_STDDEV =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0x9,
  VX_KERNEL_UNIBO_MAX3x3 =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0xA,
  VX_KERNEL_UNIBO_MAX7x7 =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0xB,
  VX_KERNEL_UNIBO_NONMAX =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0xC,
  VX_KERNEL_UNIBO_FAST9 =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0xD,
  VX_KERNEL_UNIBO_INTEGRALWINDOW =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0xE,
  VX_KERNEL_UNIBO_DISPARITY =
      VX_KERNEL_BASE(VX_ID_DEFAULT, VX_LIBRARY_RETINA) + 0xF,
};
//! [KERNEL ENUM]

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief [Graph] Computes the complementary of an image.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [out] output The output image.
 */
vx_node vxComplementaryNode(vx_graph graph, vx_image input, vx_image output);

/*! \brief [Immediate] Immediate mode version of the Complementary node.
 * \param [in] input The input image.
 * \param [out] output The output image.
 */
vx_status vxuComplementary(vx_image input, vx_image output);

/*! \brief [Graph] Computes the normalization of an image.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] min maximum pixel value in image.
 * \param [in] max minimum pixel valur in image.
 * \param [out] output The output image.
 */
vx_node vxNormalizeNode(vx_graph graph, vx_image input, vx_scalar min,
                        vx_scalar max, vx_image output);

/*! \brief [Immediate] Immediate mode version of the Normalize node.
 * \param [in] input The input image.
 * \param [in] min maximum pixel value in image.
 * \param [in] max minimum pixel valur in image.
 * \param [out] output The output image.
 */
vx_status vxuNormalize(vx_image input, vx_scalar min, vx_scalar max,
                       vx_image output);

/*! \brief [Graph] Computes the 5x5 Gaussian of an image.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] min maximum pixel value in image.
 * \param [in] max minimum pixel valur in image.
 * \param [out] output The output image.
 */
vx_node vxGaussian5x5Node(vx_graph graph, vx_image input, vx_scalar sigma,
                          vx_image output);

/*! \brief [Immediate] Immediate mode version of the 5x5 Gaussian node.
 * \param [in] input The input image.
 * \param [in] min maximum pixel value in image.
 * \param [in] max minimum pixel valur in image.
 * \param [out] output The output image.
 */
vx_status vxuGaussian5x5(vx_image input, vx_scalar sigma, vx_image output);

/*! \brief [Graph] Computes the "threshold to zero" of an image.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] th  threshold.
 * \param [out] output The output image.
 */
vx_node vxThresholdToZeroNode(vx_graph graph, vx_image input, vx_scalar th,
                              vx_image output);

/*! \brief [Immediate] Immediate mode version of the "threshold to zero" node.
 * \param [in] input The input image.
 * \param [in] th  threshold.
 * \param [out] output The output image.
 */
vx_status vxuThresholdToZero(vx_image input, vx_scalar th, vx_image output);

/*! \brief [Graph] Computes the "truncate threshold" of an image.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] th  threshold.
 * \param [out] output The output image.
 */
vx_node vxTruncateNode(vx_graph graph, vx_image input, vx_scalar th,
                       vx_image output);

/*! \brief [Immediate] Immediate mode version of the "truncate threshold" node.
 * \param [in] input The input image.
 * \param [in] th  threshold.
 * \param [out] output The output image.
 */
vx_status vxuTruncate(vx_image input, vx_scalar th, vx_image output);

/*! \brief [Graph] Add a scalar to the pixels in a image.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] value value to add.
 * \param [out] output The output image.
 */
vx_node vxAddScalarNode(vx_graph graph, vx_image input, vx_scalar value,
                        vx_scalar scale, vx_image output);

/*! \brief [Immediate] Immediate mode version of the add scalar node.
 * \param [in] input The input image.
 * \param [in] value value to add.
 * \param [out] output The output image.
 */
vx_status vxuAddScalar(vx_image input, vx_scalar value, vx_scalar scale,
                       vx_image output);

/*! \brief [Graph] Divide two images pixelwise..
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] value value to add.
 * \param [out] output The output image.
 */
vx_node vxDivideNode(vx_graph graph, vx_image input1, vx_image input2,
                     vx_image output);

/*! \brief [Immediate] Immediate mode version of the divide node.
 * \param [in] input The input image.
 * \param [in] value value to add.
 * \param [out] output The output image.
 */
vx_status vxuDivide(vx_image input1, vx_image input2, vx_image output);

/*! \brief [Graph] Compute the sigmoid function on each pixel.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] mean image mean
 * \param [in] std  image standard dev
 * \param [in] slope  sigmoid slope
 * \param [in] minRange  minimum value
 * \param [in] maxRange  maximum value
 * \param [out] output The output image.
 */
vx_node vxSigmoidNode(vx_graph graph, vx_image input, vx_scalar mean,
                      vx_scalar std, vx_scalar slope, vx_scalar minRange,
                      vx_scalar maxRange, vx_image output);

/*! \brief [Immediate] Immediate mode version of the sigmoid node.
 * \param [in] input The input image.
 * \param [in] mean image mean
 * \param [in] std  image standard dev
 * \param [in] slope  sigmoid slope
 * \param [in] minRange  minimum value
 * \param [in] maxRange  maximum value
 * \param [out] output The output image.
 */
vx_status vxuSigmoid(vx_image input, vx_scalar mean, vx_scalar std,
                     vx_scalar slope, vx_scalar minRange, vx_scalar maxRange,
                     vx_image output);

/*! \brief [Graph] Compute the mean value.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [out] mean image mean value
 */
vx_node vxMeanNode(vx_graph graph, vx_image input, vx_scalar mean);

/*! \brief [Immediate] Immediate mode version of the mean node.
 * \param [in] input The input image.
 * \param [out] mean image mean value
 */
vx_status vxuMean(vx_image input, vx_scalar mean);

/*! \brief [Graph] Compute the standard deviation value.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] mean image mean value
 * \param [out] std  image standard dev
 */
vx_node vxStdDevNode(vx_graph graph, vx_image input, vx_scalar mean,
                     vx_scalar std);

/*! \brief [Immediate] Immediate mode version of the stddev node.
 * \param [in] input The input image.
 * \param [in] mean image mean value
 * \param [out] std  image standard dev
 */
vx_status vxuStdDev(vx_image input, vx_scalar mean, vx_scalar std);

/*! \brief [Graph] Find the maximum in a 3x3 window.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [out] output The output image.
 */
vx_node vxMax3x3Node(vx_graph graph, vx_image input, vx_image output);

/*! \brief [Immediate] Immediate mode version of the max3x3 node.
 * \param [in] input The input image.
 * \param [out] output The output image.
 */
vx_status vxuMax3x3(vx_image input, vx_image output);

/*! \brief [Graph] Find the maximum in a 3x3 window.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [out] output The output image.
 */
vx_node vxMax7x7Node(vx_graph graph, vx_image input, vx_image output);

/*! \brief [Immediate] Immediate mode version of the max3x3 node.
 * \param [in] input The input image.
 * \param [out] output The output image.
 */
vx_status vxuMax7x7(vx_image input, vx_image output);

/*! \brief [Graph] Execute non-maxima suppression.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] max the image with max values.
 * \param [out] output The output image.
 */
vx_node vxNonMaxSuppression2Node(vx_graph graph, vx_image input, vx_image max,
                                 vx_image output);

/*! \brief [Immediate] Immediate mode version of the non-maxima suppressione
 * node. \param [in] input The input image. \param [in] max The image with max
 * values \param [out] output The output image.
 */
vx_status vxuNonMaxSuppression2(vx_image input, vx_image max, vx_image output);

/*! \brief [Graph] Computes FAST9 corner detector.
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] th Threshold use by the algorithm.
 * \param [out] output The output image.
 */
vx_node vxFAST9Node(vx_graph graph, vx_image input, vx_scalar th,
                    vx_image output);

/*! \brief [Immediate] Immediate mode version of FAST9 kernel.
 * \param [in] input The input image.
 * \param [in] th Threshold use by the algorithm.
 * \param [out] output The output image.
 */
vx_status vxuFAST9(vx_image input, vx_scalar th, vx_image output);

/*! \brief [Graph] Computes the integral for a window
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [in] windowSize
 * \param [out] output The output image.
 */
vx_node vxIntegralWindowNode(vx_graph graph, vx_image input,
                             vx_scalar windowSize, vx_image output);

/*! \brief [Immediate] Immediate mode version of IntegralWindow kernel.
 * \param [in] input The input image.
 * \param [in] windowSize
 * \param [out] output The output image.
 */
vx_status vxuIntegralWindow(vx_image input, vx_scalar windowSize,
                            vx_image output);

/*! \brief [Graph] Computes the integral for a window
 * \param [in] graph The handle to the graph in which to instantiate the node.
 * \param [in] input The input image.
 * \param [inout] minVals Minimum ref value for disparity.
 * \param [in] level Disparity level
 * \param [out] output The output image (disparity level).
 */
vx_node vxDisparityNode(vx_graph graph, vx_image input, vx_image minVals,
                        vx_image current, vx_scalar level, vx_image minOut,
                        vx_image output);

/*! \brief [Immediate] Immediate mode version of IntegralWindow kernel.
 * \param [in] input The input image.
 * \param [inout] minVals Minimum ref value for disparity.
 * \param [in] level Disparity level
 * \param [out] output The output image (disparity level).
 */
vx_status vxuDisparity(vx_image input, vx_image minVals, vx_image current,
                       vx_scalar level, vx_image minOut, vx_image output);

#ifdef __cplusplus
}
#endif

#endif
