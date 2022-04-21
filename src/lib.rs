use robot_ik_base::IKSolver;

extern crate libc;
use std::os::raw::c_char;
use std::ffi::CStr;
use std::str;
use std::error::Error;


extern crate k;
use k::{nalgebra as na, Isometry3};
use na::{Quaternion, UnitQuaternion, Translation3};


fn parse_c_str(s: *const c_char) -> Result<&'static str, Box<dyn Error>> {
    Ok(unsafe { CStr::from_ptr(s).to_str()? })
}


#[no_mangle]
pub extern "C" fn new_solver(urdf_ptr: *const c_char, ee_frame_ptr: *const c_char) -> *const IKSolver {
    let urdf = parse_c_str(urdf_ptr).unwrap();
    let ee_frame = parse_c_str(ee_frame_ptr).unwrap();

    let mut solver = IKSolver::from_urdf_str(urdf);
    solver.set_ee(ee_frame);

    
    Box::into_raw(Box::new(solver))
}

#[no_mangle]
pub extern "C" fn set_self_collision(iksolver: *mut IKSolver, self_collision: bool) {
    unsafe { 
        match iksolver.as_mut() {
            Some(solver) => solver.self_collision = self_collision,
            None => { }
        }
    };
}

#[no_mangle]
pub extern "C" fn solve(iksolver: *mut IKSolver, current_q_ptr: *mut f32, trans_ptr: *const [f32; 7], q_ptr: *mut f32) -> bool {
    
    let iksolver = unsafe { 
        match iksolver.as_mut() {
            Some(solver) => solver,
            None => { return false; }
        }
    };
    let current_q;
    let trans;
    unsafe {
        let dof = iksolver.dof();
        if dof < 0 {
            return false;
        }
        current_q = Vec::from(std::slice::from_raw_parts(current_q_ptr, dof as usize));
        trans = std::ptr::read(trans_ptr);
    }
    let current_q : Vec<f64> = current_q.into_iter().map(|v| v as f64).collect();
    let trans : Vec<f64> = trans.iter().map(|&v| v as f64).collect();
    

    let x = Translation3::new(trans[0], trans[1], trans[2]);
    let rot = UnitQuaternion::from_quaternion(Quaternion::new(trans[3], trans[4], trans[5], trans[6]));
    let pose = Isometry3::from_parts(x, rot);

    iksolver.solve(&current_q, &pose).ok().and_then(|q| {
        let q_array;
        unsafe {
            q_array = std::slice::from_raw_parts_mut(q_ptr, q.len());
        };
        for i in 0..q_array.len() {
            q_array[i] = q[i] as f32;
        }
        Some(())
    }).is_some()
}


#[no_mangle]
pub extern "C" fn dof(iksolver: *mut IKSolver) -> i32 {
    match unsafe { iksolver.as_ref() } {
        Some(solver) => {
            solver.dof()
        },
        None => {
            -1
        }
    }
}


#[no_mangle]
pub extern "C" fn deallocate(ptr: *mut IKSolver) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        println!("{:?}", ptr);
        Box::from_raw(ptr);
    }
}