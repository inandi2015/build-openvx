/*!
 * \file unibo_module.c
 * \brief Extension to OpenVX standard kernels
 * \author Giuseppe Tagliavini <giuseppe.tagliavini@unibo.it>
 */

#include <VX/vx.h>
#include <VX/vx_helper.h>
#include "vx_ext_unibo.h"

#include <float.h>
#include <math.h> /* exp, tanh */

////////////////////////////////////////////////////////////////////////////////
/// ComplementaryKernel
////////////////////////////////////////////////////////////////////////////////

vx_status ComplementaryInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, 0);
    vx_image image;
    vx_fourcc fourcc = 0;
    if (vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                         sizeof(vx_image)) == VX_SUCCESS &&
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc,
                     sizeof(fourcc)) == VX_SUCCESS) {
#ifdef VX_F32_EXT
      if (fourcc == FOURCC_U8 || fourcc == FOURCC_F32)
#else
      if (fourcc == FOURCC_U8)
#endif
        status = VX_SUCCESS;
      else
        status = VX_ERROR_INVALID_VALUE;
    }
    vxReleaseParameter(&param);
  }
  return status;
}

vx_status ComplementaryOutputValidator(vx_node node, vx_uint32 index,
                                       vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 1) /* output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = format;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;
      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  return status;
}

vx_status ComplementaryKernel(vx_node node, vx_reference *parameters,
                              vx_uint32 num) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (num == 2) {
    vx_image input = (vx_image)parameters[0];
    vx_image output = (vx_image)parameters[1];
    vx_uint32 x, y;
    void *in = NULL, *out = NULL;
    vx_imagepatch_addressing_t addr_in = {0}, addr_out = {0};
    vx_rectangle rect;
    vx_fourcc format = 0;

    status = VX_SUCCESS;

    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    rect = vxGetValidRegionImage(input);
    status |= vxAccessImagePatch(input, rect, 0, &addr_in, &in);
    status |= vxAccessImagePatch(output, rect, 0, &addr_out, &out);

    for (y = 0; y < addr_in.dim_y; y += addr_in.step_y) {
      for (x = 0; x < addr_in.dim_x; x += addr_in.step_x) {
        void *srcp = vxFormatImagePatchAddress2d(in, x, y, &addr_in);
        void *dstp = vxFormatImagePatchAddress2d(out, x, y, &addr_out);
        if (format == FOURCC_U8) {
          vx_uint8 src = *(vx_uint8 *)srcp;
          *(vx_uint8 *)dstp = (vx_uint8)255 - src;
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 src = *(vx_float32 *)srcp;
          *((vx_float32 *)dstp) = 1.0f - src;
        }
#endif
      }
    }
    // write back and release
    status |= vxCommitImagePatch(output, rect, 0, &addr_out, out);
    status |= vxCommitImagePatch(input, 0, 0, &addr_in,
                                 in);  // don't write back into the input
    vxReleaseRectangle(&rect);
  }

  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// NormalizeKernel
////////////////////////////////////////////////////////////////////////////////

vx_status NormalizeInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = 0;
    param = vxGetParameterByIndex(node, 0);
    if (param) {
      vx_image image = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                       sizeof(vx_image));
      if (image) {
        vx_fourcc fourcc = 0;
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc, sizeof(fourcc));
#ifdef VX_F32_EXT
        if (fourcc == FOURCC_U8 || fourcc == FOURCC_F32)
#else
        if (fourcc == FOURCC_U8)
#endif
          status = VX_SUCCESS;
        else
          status = VX_ERROR_INVALID_VALUE;
        vxReleaseImage(&image);
      }
      vxReleaseParameter(&param);
    }
  } else if ((index == 1) || (index == 2)) /* min / max values */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_parameter param0 = vxGetParameterByIndex(node, 0);
      vx_image input = 0;
      vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_scalar scalar = 0;
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                         sizeof(scalar));
        if (scalar) {
          vx_fourcc format;
          vx_enum type = VX_TYPE_INVALID;
          vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format,
                       sizeof(format));
          vx_enum typeScalar = VX_TYPE_INVALID;
          vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &typeScalar,
                        sizeof(typeScalar));
          switch (format) {
            case FOURCC_U8:
              type = VX_TYPE_UINT8;
              break;
#ifdef VX_F32_EXT
            case FOURCC_F32:
              type = VX_TYPE_FLOAT32;
              break;
#endif
            default:
              type = VX_TYPE_INVALID;
              break;
          }
          if (type != VX_TYPE_INVALID && type == typeScalar) {
            status = VX_SUCCESS;
          }
          vxReleaseScalar(&scalar);
        }

        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param0);
      vxReleaseParameter(&param);
    }
  }

  return status;
}

vx_status NormalizeOutputValidator(vx_node node, vx_uint32 index,
                                   vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 3) /* output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = format;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;
      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  return status;
}

vx_status NormalizeKernel(vx_node node, vx_reference *parameters,
                          vx_uint32 num) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (num == 4) {
    vx_image input = (vx_image)parameters[0];
    vx_scalar min = (vx_scalar)parameters[1];
    vx_scalar max = (vx_scalar)parameters[2];
    vx_image output = (vx_image)parameters[3];

    void *in = 0, *out = 0;
    vx_uint32 y, x;
    vx_imagepatch_addressing_t addr1 = {0}, addr2 = {0};
    vx_rectangle rect = 0;
    vx_fourcc format = 0;
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

    status = VX_SUCCESS;

    rect = vxGetValidRegionImage(input);
    status |= vxAccessImagePatch(input, rect, 0, &addr1, &in);
    status |= vxAccessImagePatch(output, rect, 0, &addr2, &out);

    for (y = 0; y < addr1.dim_y; y += addr1.step_y) {
      for (x = 0; x < addr1.dim_x; x += addr1.step_x) {
        void *srcp = vxFormatImagePatchAddress2d(in, x, y, &addr1);
        void *dstp = vxFormatImagePatchAddress2d(out, x, y, &addr2);
#ifdef VX_F32_EXT
        if (format == FOURCC_F32) {
          vx_float32 minVal = 0.0f, maxVal = 0.0f;
          status |= vxAccessScalarValue(min, &minVal);
          status |= vxAccessScalarValue(max, &maxVal);
          vx_float32 scale = 1.0f / (maxVal - minVal);
          vx_float32 shift = -(minVal * scale);
          vx_float32 src = *(vx_float32 *)srcp;
          *(vx_float32 *)dstp = scale * src + shift;
        } else
#endif
            if (format == FOURCC_U8) {
          vx_uint8 minVal = 0, maxVal = 0;
          status |= vxAccessScalarValue(min, &minVal);
          status |= vxAccessScalarValue(max, &maxVal);
          vx_float32 scale = 1.0f / ((vx_float32)maxVal - (vx_float32)minVal);
          vx_float32 shift = -(((vx_float32)minVal) * scale);
          vx_uint8 src = *(vx_uint8 *)srcp;
          *(vx_uint8 *)dstp = (vx_uint8)(scale * ((vx_float32)src) + shift);
        }
      }
    }
    // write back and release
    status |= vxCommitImagePatch(output, rect, 0, &addr2, out);
    status |= vxCommitImagePatch(input, 0, 0, &addr1,
                                 in);  // don't write back into the input
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// Gaussian5x5Kernel
////////////////////////////////////////////////////////////////////////////////

vx_status Gaussian5x5InputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* Input image */
  {
    vx_parameter param = 0;
    param = vxGetParameterByIndex(node, 0);
    if (param) {
      vx_image image = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                       sizeof(vx_image));
      if (image) {
        vx_fourcc fourcc = 0;
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc, sizeof(fourcc));
#ifdef VX_F32_EXT
        if (fourcc == FOURCC_U8 || fourcc == FOURCC_F32)
#else
        if (fourcc == FOURCC_U8)
#endif
          status = VX_SUCCESS;
        else
          status = VX_ERROR_INVALID_VALUE;
        vxReleaseImage(&image);
      }
      vxReleaseParameter(&param);
    }
  } else if (index == 1) /* Sigma */
  {
    vx_parameter param = 0;
    param = vxGetParameterByIndex(node, 1);
    if (param) {
      vx_scalar scalar = 0;
      vx_enum type = VX_TYPE_INVALID;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                       sizeof(scalar));
      vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
      if (scalar) {
        if (type == VX_TYPE_FLOAT32) {
          status = VX_SUCCESS;
        }
        vxReleaseScalar(&scalar);
      }
    }
    vxReleaseParameter(&param);
  }
  return status;
}

vx_status Gaussian5x5OutputValidator(vx_node node, vx_uint32 index,
                                     vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 2) /* Output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = format;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;
      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  return status;
}

void computeCoefficients(vx_float32 *coeff, vx_float32 sigma) {
  int i, j, idx = 0;
  vx_float32 sum = 0.0f, scale = -0.5f / (sigma * sigma);
  for (j = -2; j <= 2; ++j) {
    for (i = -2; i <= 2; ++i) {
      coeff[idx] = exp((vx_float32)(i * i + j * j) * scale);
      sum += coeff[idx++];
    }
  }
  for (i = 0; i < 25; ++i) coeff[i] /= sum;
}

vx_status Gaussian5x5Kernel(vx_node node, vx_reference *parameters,
                            vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image src = (vx_image)parameters[0];
    vx_scalar sigma_param = (vx_scalar)parameters[1];
    vx_image dst = (vx_image)parameters[2];
    vx_uint32 y, x;
    void *src_base = NULL;
    void *dst_base = NULL;
    vx_imagepatch_addressing_t src_addr = {0}, dst_addr = {0};
    vx_rectangle rect;
    vx_border_mode_t borders = {VX_BORDER_MODE_UNDEFINED, 0};
    vx_fourcc format = 0;
    vx_float32 sigma = 0.0f;

    status = VX_SUCCESS;
    status |=
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    status |= vxAccessScalarValue(sigma_param, &sigma);
    if (sigma == 0.0f)
      sigma = 1.1f;  // OpenCV formula: ((n-1)*0.5 - 1)*0.3 + 0.8
    rect = vxGetValidRegionImage(src);
    status |= vxAccessImagePatch(src, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst, rect, 0, &dst_addr, &dst_base);
    status |= vxQueryNode(node, VX_NODE_ATTRIBUTE_BORDER_MODE, &borders,
                          sizeof(borders));

    /*! \todo Implement other border modes */
    if (borders.mode == VX_BORDER_MODE_UNDEFINED) {
      /* shrink the image by 2 */
      vxAlterRectangle(rect, 2, 2, -2, -2);

      for (y = 2; (y < (src_addr.dim_y - 2)) && (status == VX_SUCCESS); y++) {
        for (x = 2; x < (src_addr.dim_x - 2); x++) {
          void *srcp[25] = {
              vxFormatImagePatchAddress2d(src_base, x - 2, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 2, &src_addr)};
          void *dst = vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          vx_float32 coeffs[25];
          vx_float32 sum = 0.0f;
          vx_uint32 i;
          computeCoefficients(coeffs, sigma);

          if (format == FOURCC_U8) {
            for (i = 0; i < 25; ++i) {
              sum += (vx_float32)(*((vx_uint8 *)(srcp[i]))) * coeffs[i];
            }
            *((vx_uint8 *)(dst)) = (vx_uint8)sum;
          }
#ifdef VX_F32_EXT
          else if (format == FOURCC_F32) {
            for (i = 0; i < 25; ++i) {
              sum += *((vx_float32 *)(srcp[i])) * coeffs[i];
            }
            *((vx_float32 *)(dst)) = sum;
          }
#endif
        }
      }
    } else {
      status = VX_ERROR_NOT_IMPLEMENTED;
    }
    status |= vxCommitImagePatch(src, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// ThresholdToZeroKernel / TruncateKernel
////////////////////////////////////////////////////////////////////////////////

vx_status ThresholdToZeroInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_image input = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_fourcc format = 0;
        vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
#ifdef VX_F32_EXT
        if (format == FOURCC_U8 || format == FOURCC_F32)
#else
        if (format == FOURCC_U8)
#endif
        {
          status = VX_SUCCESS;
        } else {
          status = VX_ERROR_INVALID_FORMAT;
        }
        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param);
    }
  } else if (index == 1) /*  threshold value */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_parameter param0 = vxGetParameterByIndex(node, 0);
      vx_image input = 0;
      vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_scalar scalar = 0;
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                         sizeof(scalar));
        if (scalar) {
          vx_fourcc format;
          vx_enum type = VX_TYPE_INVALID;
          vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format,
                       sizeof(format));
          vx_enum typeScalar = VX_TYPE_INVALID;
          vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &typeScalar,
                        sizeof(typeScalar));
          switch (format) {
            case FOURCC_U8:
              type = VX_TYPE_UINT8;
              break;
#ifdef VX_F32_EXT
            case FOURCC_F32:
              type = VX_TYPE_FLOAT32;
              break;
#endif
            default:
              type = VX_TYPE_INVALID;
              break;
          }
          if (type != VX_TYPE_INVALID && type == typeScalar) {
            status = VX_SUCCESS;
          }
          vxReleaseScalar(&scalar);
        }

        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param0);
      vxReleaseParameter(&param);
    }
  }
  return status;
}

vx_status ThresholdToZeroOutputValidator(vx_node node, vx_uint32 index,
                                         vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 2) /* output image */
  {
    vx_parameter src_param = vxGetParameterByIndex(node, 0);
    if (src_param) {
      vx_image src = 0;
      vxQueryParameter(src_param, VX_PARAMETER_ATTRIBUTE_REF, &src,
                       sizeof(src));
      if (src) {
        vx_uint32 width = 0, height = 0;
        vx_fourcc format = 0;

        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(height));
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

        /* fill in the meta data with the attributes so that the checker will
         * pass */
        ptr->type = VX_TYPE_IMAGE;
        ptr->dim.image.format = format;
        ptr->dim.image.width = width;
        ptr->dim.image.height = height;
        status = VX_SUCCESS;
        vxReleaseImage(&src);
      }
      vxReleaseParameter(&src_param);
    }
  }
  return status;
}

vx_status ThresholdToZeroKernel(vx_node node, vx_reference *parameters,
                                vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image src_image = (vx_image)parameters[0];
    vx_scalar threshold = (vx_scalar)parameters[1];
    vx_image dst_image = (vx_image)parameters[2];
    vx_rectangle rect;
    vx_imagepatch_addressing_t src_addr, dst_addr;
    void *src_base = NULL, *dst_base = NULL;
    vx_uint32 y = 0, x = 0;
    vx_uint8 value8 = 0;
    vx_float32 value32 = 0.0f;
    vx_fourcc format = 0;

    vxQueryImage(src_image, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    if (format == FOURCC_U8)
      vxAccessScalarValue(threshold, &value8);
    else
      vxAccessScalarValue(threshold, &value32);

    rect = vxGetValidRegionImage(src_image);
    status = VX_SUCCESS;
    status |= vxAccessImagePatch(src_image, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst_image, rect, 0, &dst_addr, &dst_base);
    for (y = 0; y < src_addr.dim_y; y++) {
      for (x = 0; x < src_addr.dim_x; x++) {
        if (format == FOURCC_U8) {
          vx_uint8 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_uint8 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          if (*src_ptr > value8)
            *dst_ptr = *src_ptr;
          else
            *dst_ptr = 0;
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_float32 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          if (*src_ptr > value32)
            *dst_ptr = *src_ptr;
          else
            *dst_ptr = 0;
        }
#endif
      }
    }

    status |= vxCommitImagePatch(src_image, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst_image, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

vx_status TruncateKernel(vx_node node, vx_reference *parameters,
                         vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image src_image = (vx_image)parameters[0];
    vx_scalar threshold = (vx_scalar)parameters[1];
    vx_image dst_image = (vx_image)parameters[2];
    vx_rectangle rect;
    vx_imagepatch_addressing_t src_addr, dst_addr;
    void *src_base = NULL, *dst_base = NULL;
    vx_uint32 y = 0, x = 0;
    vx_uint8 value8 = 0;
    vx_float32 value32 = 0.0f;
    vx_fourcc format = 0;

    vxQueryImage(src_image, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    if (format == FOURCC_U8)
      vxAccessScalarValue(threshold, &value8);
    else
      vxAccessScalarValue(threshold, &value32);

    rect = vxGetValidRegionImage(src_image);
    status = VX_SUCCESS;
    status |= vxAccessImagePatch(src_image, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst_image, rect, 0, &dst_addr, &dst_base);
    for (y = 0; y < src_addr.dim_y; y++) {
      for (x = 0; x < src_addr.dim_x; x++) {
        if (format == FOURCC_U8) {
          vx_uint8 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_uint8 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          if (*src_ptr > value8)
            *dst_ptr = value8;
          else
            *dst_ptr = *src_ptr;
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_float32 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          if (*src_ptr > value32)
            *dst_ptr = value32;
          else
            *dst_ptr = *src_ptr;
        }
#endif
      }
    }

    status |= vxCommitImagePatch(src_image, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst_image, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// AddScalarKernel
////////////////////////////////////////////////////////////////////////////////

vx_status AddScalarInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_image input = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_fourcc format = 0;
        vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
#ifdef VX_F32_EXT
        if (format == FOURCC_U8 || format == FOURCC_F32)
#else
        if (format == FOURCC_U8)
#endif
        {
          status = VX_SUCCESS;
        } else {
          status = VX_ERROR_INVALID_FORMAT;
        }
        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param);
    }
  } else if (index == 1 || index == 2) /* scalar values*/
  {
    vx_parameter src_param = vxGetParameterByIndex(node, 0);
    if (src_param) {
      vx_image src = 0;
      vxQueryParameter(src_param, VX_PARAMETER_ATTRIBUTE_REF, &src,
                       sizeof(src));
      if (src) {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param) {
          vx_scalar scalar = 0;
          vx_fourcc format = 0;

          vx_enum type = VX_TYPE_INVALID;
          vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
          vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                           sizeof(scalar));
          vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
          if (type == VX_TYPE_UINT8 && format == FOURCC_U8) {
            status = VX_SUCCESS;
          }
#ifdef VX_F32_EXT
          else if (type == VX_TYPE_FLOAT32 && format == FOURCC_F32) {
            status = VX_SUCCESS;
          }
#endif
          vxReleaseParameter(&param);
        }

        vxReleaseImage(&src);
      }
      vxReleaseParameter(&src_param);
    }
  }
  return status;
}

vx_status AddScalarOutputValidator(vx_node node, vx_uint32 index,
                                   vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 3) /* output image */
  {
    vx_parameter src_param = vxGetParameterByIndex(node, 0);
    if (src_param) {
      vx_image src = 0;
      vxQueryParameter(src_param, VX_PARAMETER_ATTRIBUTE_REF, &src,
                       sizeof(src));
      if (src) {
        vx_uint32 width = 0, height = 0;
        vx_fourcc format = 0;

        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(height));
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

        /* fill in the meta data with the attributes so that the checker will
         * pass */
        ptr->type = VX_TYPE_IMAGE;
        ptr->dim.image.format = format;
        ptr->dim.image.width = width;
        ptr->dim.image.height = height;
        status = VX_SUCCESS;
        vxReleaseImage(&src);
      }
      vxReleaseParameter(&src_param);
    }
  }
  return status;
}

vx_status AddScalarKernel(vx_node node, vx_reference *parameters,
                          vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 4) {
    vx_image src_image = (vx_image)parameters[0];
    vx_scalar value = (vx_scalar)parameters[1];
    vx_scalar factor = (vx_scalar)parameters[2];
    vx_image dst_image = (vx_image)parameters[3];
    vx_rectangle rect;
    vx_imagepatch_addressing_t src_addr, dst_addr;
    void *src_base = NULL, *dst_base = NULL;
    vx_uint32 y = 0, x = 0;
    vx_fourcc format = 0;

    vxQueryImage(src_image, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

    rect = vxGetValidRegionImage(src_image);
    status = VX_SUCCESS;
    status |= vxAccessImagePatch(src_image, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst_image, rect, 0, &dst_addr, &dst_base);
    for (y = 0; y < src_addr.dim_y; y++) {
      for (x = 0; x < src_addr.dim_x; x++) {
        if (format == FOURCC_U8) {
          vx_uint8 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_uint8 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          vx_uint8 val = 0, f = 0;
          vxAccessScalarValue(value, &val);
          vxAccessScalarValue(factor, &f);

          *dst_ptr = *src_ptr + val * f;
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_float32 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          vx_float32 val = 0.0f, f = 0.0f;
          vxAccessScalarValue(value, &val);
          vxAccessScalarValue(factor, &f);

          *dst_ptr = *src_ptr + val * f;
        }
#endif
      }
    }

    status |= vxCommitImagePatch(src_image, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst_image, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// DivideKernel
////////////////////////////////////////////////////////////////////////////////

vx_status DivideInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image (dividend) */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_image input = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_fourcc format = 0;
        vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
#ifdef VX_F32_EXT
        if (format == FOURCC_U8 || format == FOURCC_F32)
#else
        if (format == FOURCC_U8)
#endif
          status = VX_SUCCESS;

        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param);
    }
  } else if (index == 1) /* input image (divisor) */
  {
    vx_image images[2];
    vx_parameter param[2] = {
        vxGetParameterByIndex(node, 0),
        vxGetParameterByIndex(node, 1),
    };
    if (param[1]) {
      vxQueryParameter(param[0], VX_PARAMETER_ATTRIBUTE_REF, &images[0],
                       sizeof(images[0]));
      vxQueryParameter(param[1], VX_PARAMETER_ATTRIBUTE_REF, &images[1],
                       sizeof(images[1]));
      if (images[0] && images[1]) {
        vx_uint32 width[2], height[2];
        vx_fourcc format1;

        vxQueryImage(images[0], VX_IMAGE_ATTRIBUTE_WIDTH, &width[0],
                     sizeof(width[0]));
        vxQueryImage(images[1], VX_IMAGE_ATTRIBUTE_WIDTH, &width[1],
                     sizeof(width[1]));
        vxQueryImage(images[0], VX_IMAGE_ATTRIBUTE_HEIGHT, &height[0],
                     sizeof(height[0]));
        vxQueryImage(images[1], VX_IMAGE_ATTRIBUTE_HEIGHT, &height[1],
                     sizeof(height[1]));
        vxQueryImage(images[1], VX_IMAGE_ATTRIBUTE_FORMAT, &format1,
                     sizeof(format1));
        if (width[0] == width[1] && height[0] == height[1] &&
#ifdef VX_F32_EXT
            (format1 == FOURCC_U8 || format1 == FOURCC_F32))
#else
            (format1 == FOURCC_U8))
#endif
          status = VX_SUCCESS;
        vxReleaseImage(&images[0]);
        vxReleaseImage(&images[1]);
      }
      vxReleaseParameter(&param[1]);
    }
    vxReleaseParameter(&param[0]);
  }
  return status;
}

vx_status DivideOutputValidator(vx_node node, vx_uint32 index,
                                vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 2) /* output image */
  {
    /*
     * The geometry of the output image is copied from the first parameter:
     * the input images are known to match from input parameters validation.
     */
    vx_parameter param[] = {
        vxGetParameterByIndex(node, 0),
        vxGetParameterByIndex(node, 1),
        vxGetParameterByIndex(node, 2),
    };
    if (param[0] && param[1] && param[2]) {
      vx_image images[3];
      vxQueryParameter(param[0], VX_PARAMETER_ATTRIBUTE_REF, &images[0],
                       sizeof(images[0]));
      vxQueryParameter(param[1], VX_PARAMETER_ATTRIBUTE_REF, &images[1],
                       sizeof(images[1]));
      vxQueryParameter(param[2], VX_PARAMETER_ATTRIBUTE_REF, &images[2],
                       sizeof(images[2]));
      if (images[0] && images[1] && images[2]) {
        vx_uint32 width = 0, height = 0;
        vx_fourcc informat[2] = {FOURCC_VIRT, FOURCC_VIRT};
        vx_fourcc outformat = FOURCC_VIRT;

        vxQueryImage(images[0], VX_IMAGE_ATTRIBUTE_WIDTH, &width,
                     sizeof(width));
        vxQueryImage(images[0], VX_IMAGE_ATTRIBUTE_HEIGHT, &height,
                     sizeof(height));
        vxQueryImage(images[0], VX_IMAGE_ATTRIBUTE_FORMAT, &informat[0],
                     sizeof(informat[0]));
        vxQueryImage(images[1], VX_IMAGE_ATTRIBUTE_FORMAT, &informat[1],
                     sizeof(informat[1]));
        vxQueryImage(images[2], VX_IMAGE_ATTRIBUTE_FORMAT, &outformat,
                     sizeof(outformat));

        if (informat[0] == FOURCC_U8 && informat[1] == FOURCC_U8) {
          if ((outformat == FOURCC_U8) || (outformat == FOURCC_S16)) {
            status = VX_SUCCESS;
          }
        }
#ifdef VX_F32_EXT
        else if (informat[0] == FOURCC_F32 && informat[1] == FOURCC_F32) {
          if (outformat == FOURCC_F32) {
            status = VX_SUCCESS;
          }
        }
#endif
        else {
          outformat = FOURCC_S16;
          status = VX_SUCCESS;
        }
        ptr->type = VX_TYPE_IMAGE;
        ptr->dim.image.format = outformat;
        ptr->dim.image.width = width;
        ptr->dim.image.height = height;
        vxReleaseImage(&images[0]);
        vxReleaseImage(&images[1]);
        vxReleaseImage(&images[2]);
        vxReleaseParameter(&param[2]);
      }
      vxReleaseParameter(&param[0]);
      vxReleaseParameter(&param[1]);
    }
  }

  return status;
}

vx_status DivideKernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image in0 = (vx_image)parameters[0];
    vx_image in1 = (vx_image)parameters[1];
    vx_image output = (vx_image)parameters[2];
    vx_fourcc out_format = 0;
    vx_uint32 y, x, width = 0, height = 0;
    void *dst_base = NULL;
    void *src_base[2] = {NULL, NULL};
    vx_imagepatch_addressing_t dst_addr, src_addr[2];
    vx_rectangle rect;

    vxQueryImage(output, VX_IMAGE_ATTRIBUTE_FORMAT, &out_format,
                 sizeof(out_format));
    rect = vxGetValidRegionImage(in0);
    status = VX_SUCCESS;
    status |=
        vxAccessImagePatch(in0, rect, 0, &src_addr[0], (void **)&src_base[0]);
    status |=
        vxAccessImagePatch(in1, rect, 0, &src_addr[1], (void **)&src_base[1]);
    status |=
        vxAccessImagePatch(output, rect, 0, &dst_addr, (void **)&dst_base);
    width = src_addr[0].dim_x;
    height = src_addr[0].dim_y;
    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        /* input images */
        void *src0p =
            vxFormatImagePatchAddress2d(src_base[0], x, y, &src_addr[0]);
        void *src1p =
            vxFormatImagePatchAddress2d(src_base[1], x, y, &src_addr[1]);
        void *dstp = vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
        if (out_format == FOURCC_U8) {
          /* Convert inputs to float32, perform the operation. */
          vx_float32 src0 = (vx_float32)(*(vx_uint8 *)src0p);
          vx_float32 src1 = (vx_float32)(*(vx_uint8 *)src1p);
          vx_uint8 final_result_value = (vx_uint8)(src0 / src1);

          /* NO OVERFLOW CHECK!!!  */

          *(vx_uint8 *)dstp = final_result_value;
        }
#ifdef VX_F32_EXT
        else if (out_format == FOURCC_F32) {

          /* Convert inputs to float32, perform the operation. */
          vx_float32 src0 = *(vx_float32 *)src0p;
          vx_float32 src1 = *(vx_float32 *)src1p;
          vx_float32 final_result_value = src0 / src1;

          /* NO OVERFLOW CHECK!!!  */

          *(vx_float32 *)dstp = final_result_value;
        }
#endif
      }
    }
    status |= vxCommitImagePatch(in0, 0, 0, &src_addr[0], src_base[0]);
    status |= vxCommitImagePatch(in1, 0, 0, &src_addr[1], src_base[1]);
    status |= vxCommitImagePatch(output, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// SigmoidKernel
////////////////////////////////////////////////////////////////////////////////

vx_status SigmoidInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_image input = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_fourcc format = 0;
        vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
#ifdef VX_F32_EXT
        if (format == FOURCC_U8 || format == FOURCC_F32)
#else
        if (format == FOURCC_U8)
#endif
        {
          status = VX_SUCCESS;
        } else {
          status = VX_ERROR_INVALID_FORMAT;
        }
        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param);
    }
  } else if (index >= 1 &&
             index <=
                 5) /* mean, std, slope, minRange, maxRange (float scalars) */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_scalar scalar = 0;

      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                       sizeof(scalar));
      if (scalar) {
        vx_enum type = VX_TYPE_INVALID;
        vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
        if (type == VX_TYPE_FLOAT32) {
          status = VX_SUCCESS;
        }
        vxReleaseScalar(&scalar);
      }
      vxReleaseParameter(&param);
    }
  }
  return status;
}

vx_status SigmoidOutputValidator(vx_node node, vx_uint32 index,
                                 vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 6) /* output image */
  {
    vx_parameter src_param = vxGetParameterByIndex(node, 0);
    if (src_param) {
      vx_image src = 0;
      vxQueryParameter(src_param, VX_PARAMETER_ATTRIBUTE_REF, &src,
                       sizeof(src));
      if (src) {
        vx_uint32 width = 0, height = 0;
        vx_fourcc format = 0;

        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(height));
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

        /* fill in the meta data with the attributes so that the checker will
         * pass */
        ptr->type = VX_TYPE_IMAGE;
        ptr->dim.image.format = format;
        ptr->dim.image.width = width;
        ptr->dim.image.height = height;
        status = VX_SUCCESS;
        vxReleaseImage(&src);
      }
      vxReleaseParameter(&src_param);
    }
  }
  return status;
}

vx_status SigmoidKernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 7) {
    vx_image src_image = (vx_image)parameters[0];
    vx_scalar mean_param = (vx_scalar)parameters[1];
    vx_scalar std_param = (vx_scalar)parameters[2];
    vx_scalar slope_param = (vx_scalar)parameters[3];
    vx_scalar minRange_param = (vx_scalar)parameters[4];
    vx_scalar maxRange_param = (vx_scalar)parameters[5];
    vx_image dst_image = (vx_image)parameters[6];
    vx_rectangle rect;
    vx_imagepatch_addressing_t src_addr = {0}, dst_addr = {0};
    void *src_base = NULL, *dst_base = NULL;
    vx_uint32 y = 0, x = 0;
    vx_fourcc format = 0;
    vx_float32 mean, std, slope, minRange, maxRange;
    vx_float32 alpha, halfRange, halfRangeMin;

    vxAccessScalarValue(mean_param, &mean);
    vxAccessScalarValue(std_param, &std);
    vxAccessScalarValue(slope_param, &slope);
    vxAccessScalarValue(minRange_param, &minRange);
    vxAccessScalarValue(maxRange_param, &maxRange);
    alpha = slope / std;
    halfRange = 0.5f * (maxRange - minRange);
    halfRangeMin = halfRange + minRange;
    // printf("halfRangeMin halfRange mean dev alpha %f %f %f %f %f\n",
    // halfRangeMin, halfRange, mean, std, alpha);

    vxQueryImage(src_image, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

    rect = vxGetValidRegionImage(src_image);
    status = VX_SUCCESS;
    status |= vxAccessImagePatch(src_image, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst_image, rect, 0, &dst_addr, &dst_base);
    for (y = 0; y < src_addr.dim_y; y++) {
      for (x = 0; x < src_addr.dim_x; x++) {
        if (format == FOURCC_U8) {
          vx_uint8 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_uint8 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);

          vx_float32 result =
              halfRangeMin +
              (halfRange *
               tanh((vx_float32)(alpha * ((vx_float32)(*src_ptr) - mean))));

          *dst_ptr = (vx_uint8)(result);
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 *src_ptr =
              vxFormatImagePatchAddress2d(src_base, x, y, &src_addr);
          vx_float32 *dst_ptr =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);

          vx_float32 result =
              halfRangeMin +
              (halfRange * tanh((vx_float32)(alpha * (*src_ptr - mean))));

          *dst_ptr = result;

          // printf("halfRangeMin halfRange mean alpha src %f %f %f %f %f\n",
          // halfRangeMin, halfRange, mean, alpha, *src_ptr);
        }
#endif
      }
    }

    status |= vxCommitImagePatch(src_image, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst_image, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// MeanKernel
////////////////////////////////////////////////////////////////////////////////

vx_status MeanInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) {
    vx_parameter param = vxGetParameterByIndex(node, index);
    vx_image image = 0;
    vx_fourcc format = 0;

    vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image, sizeof(image));
    if (image == 0) {
      status = VX_ERROR_INVALID_PARAMETERS;
    } else {
      status = VX_SUCCESS;
      vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
#ifdef VX_F32_EXT
      if (format != FOURCC_U8 && format != FOURCC_U16 && format != FOURCC_F32)
#else
      if (format != FOURCC_U8 && format != FOURCC_U16)
#endif
      {
        status = VX_ERROR_INVALID_PARAMETERS;
      }
      vxReleaseImage(&image);
    }
    vxReleaseParameter(&param);
  }
  return status;
}

vx_status MeanOutputValidator(vx_node node, vx_uint32 index,
                              vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 1) {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_scalar output = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &output,
                       sizeof(output));
      if (output) {
        vx_enum type = 0;
        vxQueryScalar(output, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
        if (type == VX_TYPE_FLOAT32) {
          ptr->type = VX_TYPE_SCALAR;
          ptr->dim.scalar.type = type;
          status = VX_SUCCESS;
        }
        vxReleaseScalar(&output);
      }
      vxReleaseParameter(&param);
    }
  }
  return status;
}

vx_status MeanKernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 2) {
    vx_image input = (vx_image)parameters[0];
    vx_scalar s_mean = (vx_scalar)parameters[1];
    vx_float32 mean = 0.0f, sum = 0.0f;
    vx_fourcc format = 0;
    vx_rectangle rect = 0;
    vx_imagepatch_addressing_t addrs;
    void *base_ptr = NULL;
    vx_uint32 x, y;

    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    rect = vxGetValidRegionImage(input);
    status = VX_SUCCESS;
    status |= vxAccessImagePatch(input, rect, 0, &addrs, &base_ptr);
    for (y = 0; y < addrs.dim_y; y++) {
      for (x = 0; x < addrs.dim_x; x++) {
        if (format == FOURCC_U8) {
          vx_uint8 *pixel = vxFormatImagePatchAddress2d(base_ptr, x, y, &addrs);
          sum += (vx_float32)(*pixel);
        } else if (format == FOURCC_U16) {
          vx_uint16 *pixel =
              vxFormatImagePatchAddress2d(base_ptr, x, y, &addrs);
          sum += (vx_float32)(*pixel);
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 *pixel =
              vxFormatImagePatchAddress2d(base_ptr, x, y, &addrs);
          sum += *pixel;
        }
#endif
      }
    }
    mean = sum / (addrs.dim_x * addrs.dim_y);

    status |= vxCommitScalarValue(s_mean, &mean);
    status |= vxCommitImagePatch(input, rect, 0, &addrs, base_ptr);
  } else {
    status = VX_ERROR_INVALID_PARAMETERS;
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// StdDevKernel
////////////////////////////////////////////////////////////////////////////////

vx_status StdDevInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) {
    vx_parameter param = vxGetParameterByIndex(node, index);
    vx_image image = 0;
    vx_fourcc format = 0;

    vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image, sizeof(image));
    if (image == 0) {
      status = VX_ERROR_INVALID_PARAMETERS;
    } else {
      status = VX_SUCCESS;
      vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
#ifdef VX_F32_EXT
      if (format != FOURCC_U8 && format != FOURCC_U16 && format != FOURCC_F32)
#else
      if (format != FOURCC_U8 && format != FOURCC_U16)
#endif
      {
        status = VX_ERROR_INVALID_PARAMETERS;
      }
      vxReleaseImage(&image);
    }
    vxReleaseParameter(&param);
  } else if (index == 1) /* mean  */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_scalar scalar = 0;

      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                       sizeof(scalar));
      if (scalar) {
        vx_enum type = VX_TYPE_INVALID;
        vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
        if (type == VX_TYPE_FLOAT32) {
          status = VX_SUCCESS;
        }
        vxReleaseScalar(&scalar);
      }
      vxReleaseParameter(&param);
    }
  }
  return status;
}

vx_status StdDevOutputValidator(vx_node node, vx_uint32 index,
                                vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 2) {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_scalar output = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &output,
                       sizeof(output));
      if (output) {
        vx_enum type = 0;
        vxQueryScalar(output, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
        if (type == VX_TYPE_FLOAT32) {
          ptr->type = VX_TYPE_SCALAR;
          ptr->dim.scalar.type = type;
          status = VX_SUCCESS;
        }
        vxReleaseScalar(&output);
      }
      vxReleaseParameter(&param);
    }
  }
  return status;
}

vx_status StdDevKernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image input = (vx_image)parameters[0];
    vx_scalar s_mean = (vx_scalar)parameters[1];
    vx_scalar s_stddev = (vx_scalar)parameters[2];
    vx_float32 mean, sum_diff_sqrs = 0.0f, stddev = 0.0f;
    vx_fourcc format = 0;
    vx_rectangle rect = 0;
    vx_imagepatch_addressing_t addrs;
    void *base_ptr = NULL;
    vx_uint32 x, y;

    vxAccessScalarValue(s_mean, &mean);

    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    rect = vxGetValidRegionImage(input);
    status = VX_SUCCESS;
    status |= vxAccessImagePatch(input, rect, 0, &addrs, &base_ptr);
    for (y = 0; y < addrs.dim_y; y++) {
      for (x = 0; x < addrs.dim_x; x++) {
        if (format == FOURCC_U8) {
          vx_uint8 *pixel = vxFormatImagePatchAddress2d(base_ptr, x, y, &addrs);
          sum_diff_sqrs += (vx_float32)pow((vx_float32)(*pixel) - mean, 2);
        } else if (format == FOURCC_U16) {
          vx_uint16 *pixel =
              vxFormatImagePatchAddress2d(base_ptr, x, y, &addrs);
          sum_diff_sqrs += (vx_float32)pow((vx_float32)(*pixel) - mean, 2);
        }
#ifdef VX_F32_EXT
        else if (format == FOURCC_F32) {
          vx_float32 *pixel =
              vxFormatImagePatchAddress2d(base_ptr, x, y, &addrs);
          sum_diff_sqrs += (vx_float32)pow(*pixel - mean, 2);
        }
#endif
      }
    }
    stddev = (vx_float32)sqrt(sum_diff_sqrs / (addrs.dim_x * addrs.dim_y));

    status |= vxCommitScalarValue(s_stddev, &stddev);
    status |= vxCommitImagePatch(input, rect, 0, &addrs, base_ptr);
  } else {
    status = VX_ERROR_INVALID_PARAMETERS;
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// Max3x3Kernel - Max7x7Kernel
////////////////////////////////////////////////////////////////////////////////

vx_status Max3x3InputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* Input image */
  {
    vx_parameter param = 0;
    param = vxGetParameterByIndex(node, 0);
    if (param) {
      vx_image image = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                       sizeof(vx_image));
      if (image) {
        vx_fourcc fourcc = 0;
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc, sizeof(fourcc));
#ifdef VX_F32_EXT
        if (fourcc == FOURCC_U8 || fourcc == FOURCC_F32)
#else
        if (fourcc == FOURCC_U8)
#endif
          status = VX_SUCCESS;
        else
          status = VX_ERROR_INVALID_VALUE;
        vxReleaseImage(&image);
      }
      vxReleaseParameter(&param);
    }
  }
  return status;
}

vx_status Max3x3OutputValidator(vx_node node, vx_uint32 index,
                                vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 1) /* Output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = format;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;
      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  return status;
}

vx_status Max3x3Kernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 2) {
    vx_image src = (vx_image)parameters[0];
    vx_image dst = (vx_image)parameters[1];
    vx_uint32 y, x;
    void *src_base = NULL;
    void *dst_base = NULL;
    vx_imagepatch_addressing_t src_addr = {0}, dst_addr = {0};
    vx_rectangle rect;
    vx_border_mode_t borders = {VX_BORDER_MODE_UNDEFINED, 0};
    vx_fourcc format = 0;

    status = VX_SUCCESS;
    status |=
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    rect = vxGetValidRegionImage(src);
    status |= vxAccessImagePatch(src, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst, rect, 0, &dst_addr, &dst_base);
    status |= vxQueryNode(node, VX_NODE_ATTRIBUTE_BORDER_MODE, &borders,
                          sizeof(borders));

    /*! \todo Implement other border modes */
    if (borders.mode == VX_BORDER_MODE_UNDEFINED) {
      /* shrink the image by 3 */
      vxAlterRectangle(rect, 1, 1, -1, -1);

      for (y = 1; (y < (src_addr.dim_y - 1)) && (status == VX_SUCCESS); y++) {
        for (x = 1; x < (src_addr.dim_x - 1); x++) {
          void *srcp[9] = {
              vxFormatImagePatchAddress2d(src_base, x - 1, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 1, &src_addr)};
          void *dst = vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);

          vx_uint8 i;
          if (format == FOURCC_U8) {
            vx_uint8 maxVal = 0;
            for (i = 0; i < 9; ++i) {
              vx_uint8 val = *((vx_uint8 *)(srcp[i]));
              if (val > maxVal) maxVal = val;
            }
            *((vx_uint8 *)(dst)) = maxVal;
          }
#ifdef VX_F32_EXT
          else if (format == FOURCC_F32) {
            vx_float32 maxVal = -FLT_MAX;
            for (i = 0; i < 9; ++i) {
              vx_float32 val = *((vx_float32 *)(srcp[i]));
              if (val > maxVal) maxVal = val;
            }
            *((vx_uint8 *)(dst)) = maxVal;
          }
#endif
        }
      }
    } else {
      status = VX_ERROR_NOT_IMPLEMENTED;
    }
    status |= vxCommitImagePatch(src, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

vx_status Max7x7Kernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 2) {
    vx_image src = (vx_image)parameters[0];
    vx_image dst = (vx_image)parameters[1];
    vx_uint32 y, x;
    void *src_base = NULL;
    void *dst_base = NULL;
    vx_imagepatch_addressing_t src_addr = {0}, dst_addr = {0};
    vx_rectangle rect;
    vx_border_mode_t borders = {VX_BORDER_MODE_UNDEFINED, 0};
    vx_fourcc format = 0;

    status = VX_SUCCESS;
    status |=
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    rect = vxGetValidRegionImage(src);
    status |= vxAccessImagePatch(src, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst, rect, 0, &dst_addr, &dst_base);
    status |= vxQueryNode(node, VX_NODE_ATTRIBUTE_BORDER_MODE, &borders,
                          sizeof(borders));

    /*! \todo Implement other border modes */
    if (borders.mode == VX_BORDER_MODE_UNDEFINED) {
      /* shrink the image by 3 */
      vxAlterRectangle(rect, 3, 3, -3, -3);

      for (y = 3; (y < (src_addr.dim_y - 3)) && (status == VX_SUCCESS); y++) {
        for (x = 3; x < (src_addr.dim_x - 3); x++) {
          void *srcp[49] = {
              vxFormatImagePatchAddress2d(src_base, x - 3, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 3, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 3, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 3, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 3, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 3, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 3, y + 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 2, y + 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x - 1, y + 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 0, y + 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 2, y + 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y - 3, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y - 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y - 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y + 0, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y + 1, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y + 2, &src_addr),
              vxFormatImagePatchAddress2d(src_base, x + 3, y + 3, &src_addr)};
          void *dst = vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);

          vx_uint8 i;
          if (format == FOURCC_U8) {
            vx_uint8 maxVal = 0;
            for (i = 0; i < 49; ++i) {
              vx_uint8 val = *((vx_uint8 *)(srcp[i]));
              if (val > maxVal) maxVal = val;
            }
            *((vx_uint8 *)(dst)) = maxVal;
          }
#ifdef VX_F32_EXT
          else if (format == FOURCC_F32) {
            vx_float32 maxVal = -FLT_MAX;
            for (i = 0; i < 49; ++i) {
              vx_float32 val = *((vx_float32 *)(srcp[i]));
              if (val > maxVal) maxVal = val;
            }
            *((vx_uint8 *)(dst)) = maxVal;
          }
#endif
        }
      }
    } else {
      status = VX_ERROR_NOT_IMPLEMENTED;
    }
    status |= vxCommitImagePatch(src, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// NonMaxSuppressionKernel
////////////////////////////////////////////////////////////////////////////////

vx_status NonMaxSuppressionKernel(vx_node node, vx_reference *parameters,
                                  vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image in0 = (vx_image)parameters[0];
    vx_image in1 = (vx_image)parameters[1];
    vx_image output = (vx_image)parameters[2];
    vx_fourcc out_format = 0;
    vx_uint32 y, x, width = 0, height = 0;
    void *dst_base = NULL;
    void *src_base[2] = {NULL, NULL};
    vx_imagepatch_addressing_t dst_addr, src_addr[2];
    vx_rectangle rect;

    vxQueryImage(output, VX_IMAGE_ATTRIBUTE_FORMAT, &out_format,
                 sizeof(out_format));
    rect = vxGetValidRegionImage(in0);
    status = VX_SUCCESS;
    status |=
        vxAccessImagePatch(in0, rect, 0, &src_addr[0], (void **)&src_base[0]);
    status |=
        vxAccessImagePatch(in1, rect, 0, &src_addr[1], (void **)&src_base[1]);
    status |=
        vxAccessImagePatch(output, rect, 0, &dst_addr, (void **)&dst_base);
    width = src_addr[0].dim_x;
    height = src_addr[0].dim_y;
    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        /* input images */
        void *src0p =
            vxFormatImagePatchAddress2d(src_base[0], x, y, &src_addr[0]);
        void *src1p =
            vxFormatImagePatchAddress2d(src_base[1], x, y, &src_addr[1]);
        void *dstp = vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
        if (out_format == FOURCC_U8) {
          /* Convert inputs to float32, perform the operation. */
          vx_float32 src0 = (vx_float32)(*(vx_uint8 *)src0p);
          vx_float32 src1 = (vx_float32)(*(vx_uint8 *)src1p);
          vx_uint8 final_result_value = (src0 == src1 ? src0 : 0);

          *(vx_uint8 *)dstp = final_result_value;
        }
#ifdef VX_F32_EXT
        else if (out_format == FOURCC_F32) {

          /* Convert inputs to float32, perform the operation. */
          vx_float32 src0 = *(vx_float32 *)src0p;
          vx_float32 src1 = *(vx_float32 *)src1p;
          vx_float32 final_result_value = (src0 == src1 ? src0 : 0.0f);

          *(vx_float32 *)dstp = final_result_value;
        }
#endif
      }
    }
    status |= vxCommitImagePatch(in0, 0, 0, &src_addr[0], src_base[0]);
    status |= vxCommitImagePatch(in1, 0, 0, &src_addr[1], src_base[1]);
    status |= vxCommitImagePatch(output, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// FAST9Kernel
////////////////////////////////////////////////////////////////////////////////

vx_status FAST9InputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = 0;
    param = vxGetParameterByIndex(node, 0);
    if (param) {
      vx_image image = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                       sizeof(vx_image));
      if (image) {
        vx_fourcc fourcc = 0;
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc, sizeof(fourcc));
        if (fourcc == FOURCC_U8)
          status = VX_SUCCESS;
        else
          status = VX_ERROR_INVALID_VALUE;
        vxReleaseImage(&image);
      }
      vxReleaseParameter(&param);
    }
  } else if (index == 1) /* threshold value */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_parameter param0 = vxGetParameterByIndex(node, 0);
      vx_image input = 0;
      vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_scalar scalar = 0;
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                         sizeof(scalar));
        if (scalar) {
          vx_fourcc format;
          vx_enum type = VX_TYPE_INVALID;
          vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format,
                       sizeof(format));
          vx_enum typeScalar = VX_TYPE_INVALID;
          vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &typeScalar,
                        sizeof(typeScalar));
          switch (format) {
            case FOURCC_U8:
              type = VX_TYPE_UINT8;
              break;
            default:
              type = VX_TYPE_INVALID;
              break;
          }
          if (type != VX_TYPE_INVALID && type == typeScalar) {
            status = VX_SUCCESS;
          }
          vxReleaseScalar(&scalar);
        }

        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param0);
      vxReleaseParameter(&param);
    }
  }

  return status;
}

vx_status FAST9OutputValidator(vx_node node, vx_uint32 index,
                               vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 2) /* output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = format;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;
      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  return status;
}

vx_status FAST9Kernel(vx_node node, vx_reference *parameters, vx_uint32 num) {
  vx_status status = VX_FAILURE;

  status = VX_SUCCESS;

  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// IntegralWindowKernel
////////////////////////////////////////////////////////////////////////////////

vx_status IntegralWindowInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = 0;
    param = vxGetParameterByIndex(node, 0);

    if (param) {
      vx_image image = 0;
      vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                       sizeof(vx_image));
      if (image) {
        vx_fourcc fourcc = 0;
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc, sizeof(fourcc));
        if (fourcc == FOURCC_U32)
          status = VX_SUCCESS;
        else
          status = VX_ERROR_INVALID_VALUE;
        vxReleaseImage(&image);
      }
      vxReleaseParameter(&param);
    }
  } else if (index == 1) /* window value */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_parameter param0 = vxGetParameterByIndex(node, 0);
      vx_image input = 0;
      vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_scalar scalar = 0;
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                         sizeof(scalar));
        if (scalar) {
          vx_enum type = VX_TYPE_INVALID;
          vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
          if (type == VX_TYPE_UINT8) {
            status = VX_SUCCESS;
          }
          vxReleaseScalar(&scalar);
        }

        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param0);
      vxReleaseParameter(&param);
    }
  }

  return status;
}

vx_status IntegralWindowOutputValidator(vx_node node, vx_uint32 index,
                                        vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 2) /* output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = format;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;
      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  return status;
}

#ifdef CLE_ACCEL
vx_cle_tile_info IntegralWindowTilingValidator(vx_node node, vx_uint32 index) {
  vx_cle_tile_info res;
  res.width = res.height = 1;
  res.allocationType = CLE_TRANSIENT_ALLOCATION;
  if (index == 0) {
    vx_scalar scalar = 0;
    vx_uint8 winSize = 0;

    vx_parameter param1 = vxGetParameterByIndex(node, 1);
    vxQueryParameter(param1, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                     sizeof(scalar));
    vxAccessScalarValue(scalar, &winSize);

    printf("MY WINSIZE = %d\n", winSize);
    res.x[0] = 0;
    res.x[1] = winSize;
    res.y[0] = 0;
    res.y[1] = winSize;
  } else {
    res.x[0] = res.x[1] = 0;
    res.y[0] = res.y[1] = 0;
  }
  return res;
}
#endif

vx_status IntegralWindowKernel(vx_node node, vx_reference *parameters,
                               vx_uint32 num) {
  vx_status status = VX_FAILURE;
  if (num == 3) {
    vx_image src = (vx_image)parameters[0];
    vx_scalar winSize_param = (vx_scalar)parameters[1];
    vx_image dst = (vx_image)parameters[2];
    vx_uint32 y, x;
    void *src_base = NULL;
    void *dst_base = NULL;
    vx_imagepatch_addressing_t src_addr = {0}, dst_addr = {0};
    vx_rectangle rect;
    vx_border_mode_t borders = {VX_BORDER_MODE_UNDEFINED, 0};
    vx_fourcc format = 0;
    vx_uint8 winSize = 0;

    status = VX_SUCCESS;
    status |=
        vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    status |= vxAccessScalarValue(winSize_param, &winSize);
    rect = vxGetValidRegionImage(src);
    status |= vxAccessImagePatch(src, rect, 0, &src_addr, &src_base);
    status |= vxAccessImagePatch(dst, rect, 0, &dst_addr, &dst_base);
    status |= vxQueryNode(node, VX_NODE_ATTRIBUTE_BORDER_MODE, &borders,
                          sizeof(borders));

    /*! \todo Implement other border modes */
    if (borders.mode == VX_BORDER_MODE_UNDEFINED) {
      /* shrink the image by 2 */
      vxAlterRectangle(rect, 0, 0, -winSize, -winSize);

      for (y = 0; (y < (src_addr.dim_y - winSize)) && (status == VX_SUCCESS);
           y++) {
        for (x = 0; x < (src_addr.dim_x - winSize); x++) {
          vx_uint32 *srcA = vxFormatImagePatchAddress2d(src_base, x + winSize,
                                                        y + winSize, &src_addr);
          vx_uint32 *srcB =
              vxFormatImagePatchAddress2d(src_base, x + 1, y + 1, &src_addr);
          vx_uint32 *srcC = vxFormatImagePatchAddress2d(src_base, x + 1,
                                                        y + winSize, &src_addr);
          vx_uint32 *srcD = vxFormatImagePatchAddress2d(src_base, x + winSize,
                                                        y + 1, &src_addr);
          vx_uint32 *dst =
              vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);
          *dst = *srcA + *srcB - *srcC - *srcD;
        }
      }
    } else {
      status = VX_ERROR_NOT_IMPLEMENTED;
    }
    status |= vxCommitImagePatch(src, 0, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst, rect, 0, &dst_addr, dst_base);
    vxReleaseRectangle(&rect);
  }
  return status;
}

////////////////////////////////////////////////////////////////////////////////
/// DisparityKernel
////////////////////////////////////////////////////////////////////////////////

vx_status DisparityInputValidator(vx_node node, vx_uint32 index) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 0) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    vx_image image;
    vx_fourcc fourcc = 0;
    if (vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                         sizeof(vx_image)) == VX_SUCCESS &&
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc,
                     sizeof(fourcc)) == VX_SUCCESS) {
      if (fourcc == FOURCC_U32)
        status = VX_SUCCESS;
      else
        status = VX_ERROR_INVALID_VALUE;
    }
    vxReleaseParameter(&param);
  }
  if (index == 1) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    vx_image image;
    vx_fourcc fourcc = 0;
    if (vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                         sizeof(vx_image)) == VX_SUCCESS &&
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc,
                     sizeof(fourcc)) == VX_SUCCESS) {
      if (fourcc == FOURCC_U32)
        status = VX_SUCCESS;
      else
        status = VX_ERROR_INVALID_VALUE;
    }
    vxReleaseParameter(&param);
  }
  if (index == 2) /* input image */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    vx_image image;
    vx_fourcc fourcc = 0;
    if (vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &image,
                         sizeof(vx_image)) == VX_SUCCESS &&
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, &fourcc,
                     sizeof(fourcc)) == VX_SUCCESS) {
      if (fourcc == FOURCC_U8)
        status = VX_SUCCESS;
      else
        status = VX_ERROR_INVALID_VALUE;
    }
    vxReleaseParameter(&param);
  } else if (index == 3) /* level value */
  {
    vx_parameter param = vxGetParameterByIndex(node, index);
    if (param) {
      vx_parameter param0 = vxGetParameterByIndex(node, 0);
      vx_image input = 0;
      vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &input,
                       sizeof(input));
      if (input) {
        vx_scalar scalar = 0;
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar,
                         sizeof(scalar));
        if (scalar) {
          vx_enum type = VX_TYPE_INVALID;
          vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
          if (type == VX_TYPE_UINT8) {
            status = VX_SUCCESS;
          }
          vxReleaseScalar(&scalar);
        }

        vxReleaseImage(&input);
      }
      vxReleaseParameter(&param0);
      vxReleaseParameter(&param);
    }
  }
  return VX_SUCCESS;
}

vx_status DisparityOutputValidator(vx_node node, vx_uint32 index,
                                   vx_meta_format_t *ptr) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (index == 4) /* output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      // vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      // vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format,
      // sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = FOURCC_U32;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;

      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }
  if (index == 5) /* output image */
  {
    vx_parameter in = vxGetParameterByIndex(node, 0);
    vx_image input = 0;
    vxQueryParameter(in, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(vx_image));
    if (input) {
      vx_uint32 width = 0, height = 0;
      // vx_fourcc format = FOURCC_VIRT;

      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
      vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
      // vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format,
      // sizeof(format));

      ptr->type = VX_TYPE_IMAGE;
      ptr->dim.image.format = FOURCC_U8;
      ptr->dim.image.width = width;
      ptr->dim.image.height = height;
      status = VX_SUCCESS;

      vxReleaseImage(&input);
    }
    vxReleaseParameter(&in);
  }

  return VX_SUCCESS;
}

vx_status DisparityKernel(vx_node node, vx_reference *parameters,
                          vx_uint32 num) {
  vx_status status = VX_ERROR_INVALID_PARAMETERS;
  if (num == 6) {
    vx_image input = (vx_image)parameters[0];
    vx_image prevMin = (vx_image)parameters[1];
    vx_image prevOutput = (vx_image)parameters[2];
    vx_scalar level_param = (vx_scalar)parameters[3];
    vx_image newMin = (vx_image)parameters[4];
    vx_image output = (vx_image)parameters[5];
    vx_uint32 x, y;
    void *in = NULL, *prev_min = NULL, *prev_out = NULL, *new_min = NULL,
         *out = NULL;
    vx_imagepatch_addressing_t addr_input, addr_prevMin, addr_prevOutput,
        addr_newMin, addr_output;
    vx_rectangle rect;
    vx_fourcc format = 0;
    vx_uint8 level = 0;

    status = VX_SUCCESS;
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    status |= vxAccessScalarValue(level_param, &level);
    rect = vxGetValidRegionImage(input);
    status |= vxAccessImagePatch(input, rect, 0, &addr_input, &in);
    status |= vxAccessImagePatch(prevMin, rect, 0, &addr_prevMin, &prev_min);
    status |=
        vxAccessImagePatch(prevOutput, rect, 0, &addr_prevOutput, &prev_out);
    status |= vxAccessImagePatch(newMin, rect, 0, &addr_newMin, &new_min);
    status |= vxAccessImagePatch(output, rect, 0, &addr_output, &out);

    for (y = 0; y < addr_input.dim_y; y += addr_input.step_y) {
      for (x = 0; x < addr_input.dim_x; x += addr_input.step_x) {
        vx_uint32 *srcp = vxFormatImagePatchAddress2d(in, x, y, &addr_input);
        vx_uint32 *prev_minp =
            vxFormatImagePatchAddress2d(new_min, x, y, &addr_prevMin);
        vx_uint32 *new_minp =
            vxFormatImagePatchAddress2d(new_min, x, y, &addr_newMin);
        vx_uint8 *prev_outp =
            vxFormatImagePatchAddress2d(prev_out, x, y, &addr_prevOutput);
        vx_uint8 *outp = vxFormatImagePatchAddress2d(out, x, y, &addr_output);

        if (*srcp < *prev_minp) {
          *new_minp = *srcp;
          *outp = level;
        } else {
          *new_minp = *prev_minp;
          *outp = *prev_outp;
        }
      }
    }
    // write back and release
    status |= vxCommitImagePatch(output, rect, 0, &addr_output, out);
    status |= vxCommitImagePatch(newMin, rect, 0, &addr_newMin, new_min);
    status |= vxCommitImagePatch(input, 0, 0, &addr_input,
                                 in);  // don't write back into the input
    status |= vxCommitImagePatch(prevMin, 0, 0, &addr_prevMin,
                                 prev_min);  // don't write back into the input
    status |= vxCommitImagePatch(prevOutput, 0, 0, &addr_prevOutput,
                                 prev_out);  // don't write back into the input
    vxReleaseRectangle(&rect);
  }

  return status;
}

////////////////////////////////////////////////////////////////////////////////

/*! \brief A generic initializer function.
 */
vx_status Initialize(vx_node node, vx_reference *parameters, vx_uint32 num) {
  /* no initialization of memory or resources */
  return VX_SUCCESS;
}

/*! \brief A generic deinitializer function.
 */
vx_status Deinitialize(vx_node node, vx_reference *parameters, vx_uint32 num) {
  /*  no de-initialization of memory or resources */
  return VX_SUCCESS;
}

//**********************************************************************
//  PUBLIC FUNCTION
//**********************************************************************

/*! \brief The entry point into this module to add the extensions to OpenVX.
 * \param [in] context The handle to the implementation context.
 * \return A \ref vx_status_e enumeration. Returns errors if some or all kernels
 * were not added correctly. \note This follows the function pointer definition
 * of a \ref vx_publish_kernels_f and uses the predefined name for the entry
 * point, "vxPublishKernels".
 */
vx_status vxPublishKernels(vx_context context) {
  vx_status status = VX_SUCCESS;
  vx_kernel kernel = NULL;

  /* ComplementaryKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.complementary",
                            VX_KERNEL_UNIBO_COMPLEMENTARY, ComplementaryKernel,
                            2, ComplementaryInputValidator,
                            ComplementaryOutputValidator, Initialize,
                            Deinitialize, NULL, tile, "complementary");
#else
    kernel = vxAddKernel(
        context, "it.unibo.retina.complementary", VX_KERNEL_UNIBO_COMPLEMENTARY,
        ComplementaryKernel, 2, ComplementaryInputValidator,
        ComplementaryOutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 1, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 1, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* NormalizeKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(
        context, "it.unibo.retina.normalize", VX_KERNEL_UNIBO_NORMALIZE,
        NormalizeKernel, 4, NormalizeInputValidator, NormalizeOutputValidator,
        Initialize, Deinitialize, NULL, tile, "normalization");
#else
    kernel = vxAddKernel(context, "it.unibo.retina.normalize",
                         VX_KERNEL_UNIBO_NORMALIZE, NormalizeKernel, 4,
                         NormalizeInputValidator, NormalizeOutputValidator,
                         Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 2, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 3, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 3, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* Gaussian5x5Kernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile = (vx_cle_tile_info){
        25 * sizeof(vx_float32), 0, {0, 0}, {0, 0}, CLE_STATE_SCALAR};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {2, 2}, {2, 2}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel =
        vxAddKernelCLE(context, "it.unibo.retina.gaussian5x5",
                       VX_KERNEL_UNIBO_GAUSSIAN5X5, Gaussian5x5Kernel, 3,
                       Gaussian5x5InputValidator, Gaussian5x5OutputValidator,
                       Initialize, Deinitialize, NULL, tile, "gaussian5x5");
#else
    kernel = vxAddKernel(context, "it.unibo.retina.gaussian5x5",
                         VX_KERNEL_UNIBO_GAUSSIAN5X5, Gaussian5x5Kernel, 3,
                         Gaussian5x5InputValidator, Gaussian5x5OutputValidator,
                         Initialize, Deinitialize);
#endif

    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* ThresholdToZeroKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(
        context, "it.unibo.retina.threshold_to_zero",
        VX_KERNEL_UNIBO_THRESHOLD_TO_ZERO, ThresholdToZeroKernel, 3,
        ThresholdToZeroInputValidator, ThresholdToZeroOutputValidator,
        Initialize, Deinitialize, NULL, tile, "threshold_to_zero");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.threshold_to_zero",
                    VX_KERNEL_UNIBO_THRESHOLD_TO_ZERO, ThresholdToZeroKernel, 3,
                    ThresholdToZeroInputValidator,
                    ThresholdToZeroOutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* TruncateKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.truncate",
                            VX_KERNEL_UNIBO_TRUNCATE, TruncateKernel, 3,
                            ThresholdToZeroInputValidator,   // it is the same!
                            ThresholdToZeroOutputValidator,  // it is the same!
                            Initialize, Deinitialize, NULL, tile, "truncate");
#else
    kernel = vxAddKernel(context, "it.unibo.retina.truncate",
                         VX_KERNEL_UNIBO_TRUNCATE, TruncateKernel, 3,
                         ThresholdToZeroInputValidator,   // it is the same!
                         ThresholdToZeroOutputValidator,  // it is the same!
                         Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* AddScalarKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.add_scalar",
                            VX_KERNEL_UNIBO_ADD_SCALAR, AddScalarKernel, 4,
                            AddScalarInputValidator, AddScalarOutputValidator,
                            Initialize, Deinitialize, NULL, tile, "add_scalar");
#else
    kernel = vxAddKernel(context, "it.unibo.retina.add_scalar",
                         VX_KERNEL_UNIBO_ADD_SCALAR, AddScalarKernel, 4,
                         AddScalarInputValidator, AddScalarOutputValidator,
                         Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      status = vxAddParameterToKernel(kernel, 2, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 3, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 3, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* DivideKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.divide",
                            VX_KERNEL_UNIBO_DIVIDE, DivideKernel, 3,
                            DivideInputValidator, DivideOutputValidator,
                            Initialize, Deinitialize, NULL, tile, "divide");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.divide", VX_KERNEL_UNIBO_DIVIDE,
                    DivideKernel, 3, DivideInputValidator,
                    DivideOutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 1, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* SigmoidKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.sigmoid",
                            VX_KERNEL_UNIBO_SIGMOID, SigmoidKernel, 7,
                            SigmoidInputValidator, SigmoidOutputValidator,
                            Initialize, Deinitialize, NULL, tile, "sigmoid");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.sigmoid", VX_KERNEL_UNIBO_SIGMOID,
                    SigmoidKernel, 7, SigmoidInputValidator,
                    SigmoidOutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 2, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 3, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 4, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 5, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 6, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 6, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* MeanKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile = (vx_cle_tile_info){
        NB_CORES * sizeof(vx_float32) + NB_CORES * sizeof(vx_int32),
        0,
        {0, 0},
        {0, 0},
        CLE_STATE_SCALAR};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel =
        vxAddKernelCLE(context, "it.unibo.retina.mean", VX_KERNEL_UNIBO_MEAN,
                       MeanKernel, 2, MeanInputValidator, MeanOutputValidator,
                       Initialize, Deinitialize, NULL, tile, "mean");
#else
    kernel = vxAddKernel(context, "it.unibo.retina.mean", VX_KERNEL_UNIBO_MEAN,
                         MeanKernel, 2, MeanInputValidator, MeanOutputValidator,
                         Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_OUTPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* StdDevKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile = (vx_cle_tile_info){
        NB_CORES * sizeof(vx_float32) + NB_CORES * sizeof(vx_int32),
        0,
        {0, 0},
        {0, 0},
        CLE_STATE_SCALAR};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.stddev",
                            VX_KERNEL_UNIBO_STDDEV, StdDevKernel, 3,
                            StdDevInputValidator, StdDevOutputValidator,
                            Initialize, Deinitialize, NULL, tile, "stddev");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.stddev", VX_KERNEL_UNIBO_STDDEV,
                    StdDevKernel, 3, StdDevInputValidator,
                    StdDevOutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* Max3x3Kernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {1, 1}, {1, 1}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.max3x3",
                            VX_KERNEL_UNIBO_MAX3x3, Max3x3Kernel, 2,
                            Max3x3InputValidator, Max3x3OutputValidator,
                            Initialize, Deinitialize, NULL, tile, "max3x3");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.max3x3", VX_KERNEL_UNIBO_MAX3x3,
                    Max3x3Kernel, 2, Max3x3InputValidator,
                    Max3x3OutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 1, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 1, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* Max7x7Kernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {3, 3}, {3, 3}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.max7x7",
                            VX_KERNEL_UNIBO_MAX7x7, Max7x7Kernel, 2,
                            Max3x3InputValidator, Max3x3OutputValidator,
                            Initialize, Deinitialize, NULL, tile, "max7x7");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.max7x7", VX_KERNEL_UNIBO_MAX7x7,
                    Max7x7Kernel, 2, Max3x3InputValidator,
                    Max3x3OutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 1, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 1, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* NonMaximaSuppressionKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.nonmax",
                            VX_KERNEL_UNIBO_NONMAX, NonMaxSuppressionKernel, 3,
                            DivideInputValidator, DivideOutputValidator,
                            Initialize, Deinitialize, NULL, tile, "nonmax");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.nonmax", VX_KERNEL_UNIBO_NONMAX,
                    NonMaxSuppressionKernel, 3, DivideInputValidator,
                    DivideOutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 1, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* FAST9Kernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {3, 3}, {3, 3}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.fast9",
                            VX_KERNEL_UNIBO_FAST9, FAST9Kernel, 3,
                            FAST9InputValidator, FAST9OutputValidator,
                            Initialize, Deinitialize, NULL, tile, "fast9");
#else
    kernel = vxAddKernel(
        context, "it.unibo.retina.fast9", VX_KERNEL_UNIBO_FAST9, FAST9Kernel, 3,
        FAST9InputValidator, FAST9OutputValidator, Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* IntegralWindowKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(
        context, "it.unibo.retina.integral_window",
        VX_KERNEL_UNIBO_INTEGRALWINDOW, IntegralWindowKernel, 3,
        IntegralWindowInputValidator, IntegralWindowOutputValidator, Initialize,
        Deinitialize, IntegralWindowTilingValidator, tile, "integral_window");
#else
    kernel =
        vxAddKernel(context, "it.unibo.retina.integral_window",
                    VX_KERNEL_UNIBO_INTEGRALWINDOW, IntegralWindowKernel, 3,
                    IntegralWindowInputValidator, IntegralWindowOutputValidator,
                    Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

  /* DisparityKernel */
  {
#ifdef CLE_ACCEL
    vx_cle_tile_info tile =
        (vx_cle_tile_info){0, 0, {0, 0}, {0, 0}, CLE_STATE_NONE};
    vx_cle_tile_info tileInput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    vx_cle_tile_info tileOutput =
        (vx_cle_tile_info){1, 1, {0, 0}, {0, 0}, CLE_TRANSIENT_ALLOCATION};
    kernel = vxAddKernelCLE(context, "it.unibo.retina.disparity",
                            VX_KERNEL_UNIBO_DISPARITY, DisparityKernel, 6,
                            DisparityInputValidator, DisparityOutputValidator,
                            Initialize, Deinitialize, NULL, tile, "disparity");
#else
    kernel = vxAddKernel(context, "it.unibo.retina.disparity",
                         VX_KERNEL_UNIBO_DISPARITY, DisparityKernel, 6,
                         DisparityInputValidator, DisparityOutputValidator,
                         Initialize, Deinitialize);
#endif
    if (kernel) {
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 0, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 1, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 1, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 2, VX_INPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileInput);
#else
      status = vxAddParameterToKernel(kernel, 2, VX_INPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;

      status = vxAddParameterToKernel(kernel, 3, VX_INPUT, VX_TYPE_SCALAR,
                                      VX_PARAMETER_STATE_REQUIRED);
      if (status != VX_SUCCESS) goto exit;

#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 4, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 4, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;
#ifdef CLE_ACCEL
      status =
          vxAddParameterToKernelCLE(kernel, 5, VX_OUTPUT, VX_TYPE_IMAGE,
                                    VX_PARAMETER_STATE_REQUIRED, tileOutput);
#else
      status = vxAddParameterToKernel(kernel, 5, VX_OUTPUT, VX_TYPE_IMAGE,
                                      VX_PARAMETER_STATE_REQUIRED);
#endif
      if (status != VX_SUCCESS) goto exit;

      status = vxFinalizeKernel(kernel);
      if (status != VX_SUCCESS) goto exit;
    }
  }

exit:
  if (status != VX_SUCCESS) {
    vxRemoveKernel(kernel);
  }
  return status;
}
