function best_q = UR5ikineP(ur5, T, referQ)
qGroup = UR5ikine(ur5, T);
best_q = zeros(1,6);
min_cost = inf;
for i = 1:8
    this_q = qGroup(i,:);
    if is_q_valid(this_q)
        new_cos = q_diff(referQ, this_q);
        if new_cos < min_cost
            best_q = this_q;
            min_cost = new_cos;
        end
    end
end

function cost = q_diff(q1, q2)
    dof = size(q1,2);
    cost = 0;
    for j=1:dof
        cost = cost + abs(q1(j) - q2(j));
    end
end

function isValid = is_q_valid(q)
    isValid = true;
    for k = 1:6
        if isnan(q(k))
            isValid = false;
            return;
        end
    end
end
    end