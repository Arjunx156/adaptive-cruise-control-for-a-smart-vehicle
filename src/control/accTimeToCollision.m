function ttc = accTimeToCollision(gap, vRel)
%ACCTIMETOCOLLISION Time until the gap closes at the current relative speed.
%
%   vRel is (vLead - vEgo), so a negative value means closing. When the gap is
%   opening the result is Inf.

closingSpeed = -vRel;
if closingSpeed <= 1e-6
    ttc = Inf;
else
    ttc = max(gap, 0) / closingSpeed;
end
end
