function value = get_safe(s, field, default_value)
% GET_SAFE - Safely retrieves a field from a structure.
% If the structure `s` exists, has the specified `field`, and the field's
% value is not empty, this function returns the value. Otherwise, it
% returns the `default_value`.
%
% Syntax: value = get_safe(s, field, default_value)

    if isstruct(s) && isfield(s, field) && ~isempty(s.(field))
        value = s.(field);
    else
        value = default_value;
    end
end