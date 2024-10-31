function refs = assignReferences(traj,k)

    refs.xBar = traj.x(1,k);
    refs.vxBar = traj.x(2,k);
    refs.yBar = traj.x(3,k);
    refs.vyBar = traj.x(4,k);
    refs.axBar = traj.u(1,k);
    refs.ayBar = traj.u(2,k);
    
end

