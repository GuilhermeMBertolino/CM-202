function saturatedValue = saturate(value, minimum, maximum)
% saturatedValue = saturate(value, minimum, maximum) saturates a value
% given minimum and maximum values.
    saturatedValue = max(minimum, min(value, maximum));
end