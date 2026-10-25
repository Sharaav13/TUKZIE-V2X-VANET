%% TUKZIE V2X VANET (TVV) - SYNTHETIC TEST ANALYSIS
% Runs tests/test_analysis.m, unchanged, on the SYNTHETIC logs in this folder (made by make_synthetic.py)
% test_analysis.m reads the folders next to itself, so an exact copy is placed here first (overwritten on every run)
% Output: plots/ and test_results.txt in this folder (the real results in tests/ are not touched)
% Folders with no synthetic logs (heldersig straight, howard centre, ...) report a missing-folder error

here = fileparts(mfilename('fullpath'));
copyfile(fullfile(here, '..', 'test_analysis.m'), fullfile(here, 'test_analysis.m'));
run(fullfile(here, 'test_analysis.m'));
