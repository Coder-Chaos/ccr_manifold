#pragma once
#include "xresize_accel.h"

namespace quick_inspection
{
    class hw_resize
    {
    private:
        /* data */

        // int input_width_;
        // int input_height_;
        // int out_width_;
        // int out_height_;

        XResize_accel resize_accel_;

    public:
        hw_resize(int uio_num, int in_w, int in_h, int out_w, int out_h, u64 in_addr, u64 out_addr)
        {
            int ret;

            // init hardware
            ret = XResize_accel_Initialize_With_UIO_NUM(&resize_accel_, uio_num);
            if (ret != 0)
            {
                printf("\rresize accelerator init fail!\n");
            }

            /* set parameters */
            XResize_accel_Set_rows_in(&resize_accel_, in_h);
            XResize_accel_Set_cols_in(&resize_accel_, in_w);
            XResize_accel_Set_rows_out(&resize_accel_, out_h);
            XResize_accel_Set_cols_out(&resize_accel_, out_w);
            XResize_accel_Set_img_inp(&resize_accel_, in_addr);
            XResize_accel_Set_img_out(&resize_accel_, out_addr);
        }
        hw_resize(/* args */) {}
        ~hw_resize()
        {
            // release hardware
            XResize_accel_Release(&resize_accel_);
        }

        /* check state */
        int IsIdle(void)
        {
            return XResize_accel_IsIdle(&resize_accel_);
        }

        int IsReady(void)
        {
            return XResize_accel_IsReady(&resize_accel_);
        }

        int IsDone(void)
        {

            while (!XResize_accel_IsDone(&resize_accel_))
                usleep(1);
        }

        // start hardware
        void Start(void)
        {
            XResize_accel_Start(&resize_accel_);
            while (!XResize_accel_IsDone(&resize_accel_))
                usleep(1);
        }
    };

} // namespace quick_inspection