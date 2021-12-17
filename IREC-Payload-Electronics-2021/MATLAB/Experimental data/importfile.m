function datastore = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  DATASTORE = IMPORTFILE(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the data as a table.
%
%  DATASTORE = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  payloadrolltest1 = importfile("C:\Education\OneDrive - University of Florida\Documents\Supplementary Material\Rocket Team\ACS Electronics Lead\Experimental data\payload roll test 1.csv", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 20-Nov-2021 22:52:28

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Var1", "VarName2"];
opts.SelectedVariableNames = "VarName2";
opts.VariableTypes = ["string", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% Specify variable properties
opts = setvaropts(opts, "Var1", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Var1", "EmptyFieldRule", "auto");
opts = setvaropts(opts, "VarName2", "ThousandsSeparator", ",");

% Import the data
datastore = readtable(filename, opts);

end