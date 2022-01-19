use robot_ik_base;

extern crate libc;
use std::os::raw::c_char;
use std::ffi::CStr;
use std::str;
use std::error::Error;


extern crate k;
use k::nalgebra as na;
use na::{Quaternion, UnitQuaternion, Vector3};
use std::panic;


fn parse_c_str(s: *const c_char) -> Result<&'static str, Box<dyn Error>> {
    Ok(unsafe { CStr::from_ptr(s).to_str()? })
}

#[no_mangle]
pub extern "cdecl" fn init(urdf: *const c_char, dof: i32) -> i32 {
    let urdf = parse_c_str(urdf);
    if urdf.is_err() {
        return 5;
    }
    let urdf = urdf.unwrap();
    match robot_ik_base::init(urdf, dof as usize) {
        Ok(id) => {
            id as i32
        },
        Err(_e) => {
            -1
        }
    }
}

#[no_mangle]
pub extern "cdecl" fn solve(robot_id: i32, current_q_ptr: *mut f32, frame_ptr: *const c_char, trans_ptr: *const [f32;7], q_ptr: *mut f32) -> i32 {
    let robot_id = robot_id as usize;
    let frame;
    frame = parse_c_str(frame_ptr);
    if frame.is_err() {
        return -1;
    }
    let frame = frame.unwrap();
    let dof = robot_ik_base::dof(robot_id, frame);

    let current_q;
    let trans;
    unsafe {
        current_q = Vec::from(std::slice::from_raw_parts(current_q_ptr, dof));
        trans = std::ptr::read(trans_ptr);
    }

    let x = Vector3::new(trans[0], trans[1], trans[2]);
    let rot = UnitQuaternion::from_quaternion(Quaternion::new(trans[3], trans[4], trans[5], trans[6]));
    
    let result = panic::catch_unwind(|| {
        match robot_ik_base::solve_32(robot_id, &current_q, &frame, &x, &rot) {
            Ok(q) => {
                let q_array;
                unsafe {
                    q_array = std::slice::from_raw_parts_mut(q_ptr, dof);
                };
                for i in 0..q_array.len() {
                    q_array[i] = q[i];
                }
                0
            },
            Err(e) => {
                e.code
            }
        }
    });
    if let Ok(b) = result {
        return b;
    }
    return 7;
}
