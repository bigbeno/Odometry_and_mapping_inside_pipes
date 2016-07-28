function T = qt_to_T(q_t)

    T = [quat2dcmElena(q_t(1:4)'), q_t(5:7); 0 0 0 1];
end