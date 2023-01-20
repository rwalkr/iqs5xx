//! Register IDs for IQS5xx

macro_rules! register_id {
    ($name:ident, $addr:literal) => {
        $crate::paste::paste! {
            pub const [<$name:upper>]: u16 = $addr;
        }
    };
}

register_id!(PRODUCT_NUMBER_0, 0x0000);
register_id!(GESTURE_EVENTS, 0x000D);
register_id!(SYSTEM_CONTROL_0, 0x0431);
register_id!(REPORT_RATE_ACTIVE, 0x057A);
register_id!(REPORT_RATE_IDLE_TOUCH, 0x057C);
register_id!(REPORT_RATE_IDLE, 0x057E);
register_id!(REPORT_RATE_LP1, 0x0580);
register_id!(REPORT_RATE_LP2, 0x0582);
register_id!(TIMEOUT_ACTIVE, 0x0584);
register_id!(TIMEOUT_IDLE_TOUCH, 0x0585);
register_id!(TIMEOUT_IDLE, 0x0586);
register_id!(TIMEOUT_LP1, 0x0587);
register_id!(REFERENCE_UPDATE_TIME, 0x0588);
register_id!(SNAP_TIMEOUT, 0x0589);
register_id!(I2C_TIMEOUT, 0x058A);
register_id!(SINGLE_FINGER_GESTURES, 0x06B7);
register_id!(MULTI_FINGER_GESTURES, 0x06B8);
register_id!(TAP_TIME, 0x06B9);
register_id!(TAP_DISTANCE, 0x06BB);
register_id!(HOLD_TIME, 0x06BD);
register_id!(SWIPE_INITIAL_TIME, 0x06BF);
register_id!(SWIPE_INITIAL_DISTANCE, 0x06C1);
register_id!(SWIPE_CONSECUTIVE_TIME, 0x06C3);
register_id!(SWIPE_CONSECUTIVE_DISTANCE, 0x06C5);
register_id!(SWIPE_ANGLE, 0x06C7);
register_id!(SCROLL_INITIAL_DISTANCE, 0x06C8);
register_id!(SCROLL_ANGLE, 0x06CA);
register_id!(ZOOM_INITIAL_DISTANCE, 0x06CB);
register_id!(ZOOM_ANGLE, 0x06CD);
register_id!(END_WINDOW, 0xEEEE);
