load doubleSupporttotalForceFTS;
load doubleSupporttotalForceWBDT;

time     = doubleSupporttotalForceFTS.Time;
dataFTS  = doubleSupporttotalForceFTS.Data;
dataWBDT = doubleSupporttotalForceWBDT.Data;

normsFTS  = arrayfun(@(idx) norm(dataFTS(idx,:)), 1:size(dataFTS,1))';
normsWBDT = arrayfun(@(idx) norm(dataWBDT(idx,:)), 1:size(dataFTS,1))';

subplot(121)
p1 = plot(time, dataFTS);
legend('FTS_x', 'FTS_y', 'FTS_z');

subplot(122)
p2 = plot(time, dataWBDT);
legend('WBD_x', 'WBD_y', 'WBD_z');

figure,
scatter(ones(size(normsFTS,1),1), normsFTS, 4, 'fill');
hold on;
scatter(2*ones(size(normsWBDT,1),1), normsFTS, 3, 'fill');

boxplot([normsFTS, normsWBDT]);
