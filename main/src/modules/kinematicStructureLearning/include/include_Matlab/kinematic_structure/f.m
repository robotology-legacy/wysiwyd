function obj = f(x_SVDD,y_SVDD,C,kernel_type,kernel_param_value)
    SET = incSVDD(x_SVDD,y_SVDD,C,kernel_type,kernel_param_value,0);
    sample_margin = CalSampleMargin(SET.O.x,SET.O.y,SET,kernel_param_value,kernel_type);  % calculate the sample margin
    sm_entropy = entropy(sample_margin);
    obj = sm_entropy;
end