r = createUR5_inCoppeliaSim();

%% REFERENCE
T0 = transl(0, 0, 0) * rpy2tr(0, 0, 0, 'deg');
T1 = transl(0, 0, 0.0085) * rpy2tr(0, 0, 0, 'deg');
T2 = transl(-0.0703, 0, 0.0746) * rpy2tr(0, -90, 0, 'deg');
T3 = transl(-0.0703, 0, 0.4997) * rpy2tr(0, -90, 0, 'deg');
T4 = transl(-0.0703, 0, 0.8918) * rpy2tr(0, -90, 0, 'deg');
T5 = transl(-0.11, 0, 0.9374) * rpy2tr(0, 0, 0, 'deg');
T6 = transl(-0.0956, 0, 0.9866) * rpy2tr(0, -90, 0, 'deg');

hold on;
trplot(T0, 'length', 0.1, 'rgb');
trplot(T1, 'length', 0.1, 'rgb');
trplot(T2, 'length', 0.1, 'rgb');
trplot(T3, 'length', 0.1, 'rgb');
trplot(T4, 'length', 0.1, 'rgb');
trplot(T5, 'length', 0.1, 'rgb');
trplot(T6, 'length', 0.1, 'rgb');