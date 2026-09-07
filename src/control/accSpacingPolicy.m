function dDes = accSpacingPolicy(vEgo, p)
%ACCSPACINGPOLICY Constant time gap desired separation (section 2.2).
%
%   dDes = d0 + tGap * vEgo
%
%   The target separation scales with the ego vehicle's own speed, matching how
%   drivers are taught to follow and giving the string-stability property a
%   constant-distance policy lacks. The standstill offset d0 stops the target
%   collapsing to zero when stopped.

dDes = p.spacing.standstillGap + p.spacing.timeGap * max(vEgo, 0);
end
