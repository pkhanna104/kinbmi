function [y,clipped] = clip(x,range)

% Clips values outside of the range vector to the boundary values.

y = max(x,range(1));
y = min(y,range(2));

if y ~= x
    clipped = 1;
else
    clipped = 0;
end

end
