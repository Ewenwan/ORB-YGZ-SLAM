#ifndef YGZ_ALIGN_H_
#define YGZ_ALIGN_H_

#include "Common.h"

// 有关align部分的算法
// This part is moved from rpg_SVO with modification to support ygz 

namespace ygz {

    /**
     * @brief align a pixel with reference image patch   将像素与参考图像块对齐
     * @param[in] cur_img The current image        
     * @param[in] ref_patch_with_boarder the patch with boarder, used to compute the gradient (or FEJ)
     * @param[in] ref_patch the patch in reference frame, by default is 64x64
     * @param[in] n_iter maximum iterations                 最大迭代次数
     * @param[out] cur_px_estimate the estimated position in current image, must have an initial value 估计值
     * @return True if successful
     */
    bool Align2D(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Vector2f &cur_px_estimate,
            bool no_simd = false);

}

#endif
