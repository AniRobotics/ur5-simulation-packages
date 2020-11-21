function [gst, transform_upto_joint] = forward_kinematics(gst0, joint_axes, q_axes, theta)
	dim = 3;
	num_of_joints = 6;
	gst_temp = eye(dim+1,dim+1);
	transform_upto_joint = zeros(dim+1, dim+1, num_of_joints+1);
	for i = 1:num_of_joints 
		transform_upto_joint(:,:,i) = gst_temp;
		omega = joint_axes(:,i);
		q = q_axes(:,i);
		xi = [cross(-omega, q); omega];
		gst_joint_i = exp_twist(xi, theta(i));
		gst_temp = gst_temp*gst_joint_i;
	end
	transform_upto_joint(:,:,end) = gst_temp;
	gst = gst_temp*gst0;
end