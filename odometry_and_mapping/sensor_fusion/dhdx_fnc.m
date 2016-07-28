function dhdx = dhdx_fnc(x, in_bend, joint_visible, new_pipe)

x=num2cell(x);
[x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28]=deal(x{:});

dhdx = J_CF(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28);

if joint_visible
    dhdx = [dhdx; ...
        J_VJT(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28)];
end

dhdx = [dhdx; ...
        J_PE_R_CF(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28); ...
        J_PE_pipe(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28)];

J_PE_t_CF_full = J_PE_t_CF(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28);    
if in_bend
    dhdx = [dhdx; ...
        J_PE_t_CF_full];
else
    dhdx = [dhdx; ...
        J_PE_t_CF_full(2:3,:)];
end

if new_pipe
    dhdx = [dhdx; ...
        J_PE_t_pipe()];
end
    
end


