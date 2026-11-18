function [meas, T] = load_meas_bus(csvFile)
%LOAD_MEAS_BUS  Read a Team 14 combined.csv into a struct of timeseries matching Meas_Bus.
%   [meas, T] = load_meas_bus('run_20261111_141500_walk1/combined.csv');
%   Use meas in a "From Workspace" block with Output data type = Bus: Meas_Bus.
%   T is the raw table (one row per IMU sample, ~100 Hz).
%
%   Vector elements are columns name_1..name_n in the CSV and become [n 1] signals here.

opts = detectImportOptions(csvFile);
opts = setvartype(opts, 'utc', 'string');
T = readtable(csvFile, opts);
t = T.timestamp;
N = height(T);

col  = @(name, n) T{:, compose(name + "_%d", 1:n)};          % N x n matrix
asCol = @(X) reshape(X.', [size(X, 2) 1 N]);                  % -> n x 1 x N (time last)

meas.imuAccel      = timeseries(asCol(col("imuAccel", 3)), t);
meas.imuGyro       = timeseries(asCol(col("imuGyro", 3)), t);
meas.imuValid      = timeseries(logical(T.imuValid), t);
meas.baroAltitude  = timeseries(T.baroAltitude, t);
meas.baroValid     = timeseries(logical(T.baroValid), t);
meas.lteRanges     = timeseries(asCol(col("lteRanges", 6)), t);
meas.lteValid      = timeseries(asCol(logical(col("lteValid", 6))), t);
meas.lteSourceId   = timeseries(asCol(uint16(col("lteSourceId", 6))), t);
meas.leoRangeRates = timeseries(asCol(col("leoRangeRates", 12)), t);
meas.leoValid      = timeseries(asCol(logical(col("leoValid", 12))), t);
meas.leoSourceId   = timeseries(asCol(uint16(col("leoSourceId", 12))), t);
meas.gnssPositionNED = timeseries(asCol(col("gnssPositionNED", 3)), t);
meas.gnssVelocityNED = timeseries(asCol(col("gnssVelocityNED", 3)), t);
meas.gnssValid     = timeseries(logical(T.gnssValid), t);
meas.timestamp     = timeseries(t, t);
end
