classdef AccSimulinkParityTest < matlab.unittest.TestCase
    %ACCSIMULINKPARITYTEST The Simulink model must agree with the .m simulation.
    %
    %   Two routes to the same answer are only worth having if they agree. Both
    %   integrate the same equations with the same fixed-step RK4 solver and share
    %   one control law, so a disagreement is a modelling error in one of them,
    %   not a tolerance to be widened.
    %
    %   These tests load Simulink, so they are slower than the rest of the suite.
    %   Run them alone with:  runTests('Filter', "Simulink")

    methods (TestClassSetup)
        function setUpPath(~)
            accSetupPath();
        end
    end

    methods (Test)
        function modelBuildsAndSaves(testCase)
            modelPath = buildAccSimulinkModel();
            testCase.verifyTrue(isfile(modelPath));
        end

        function leadBrakingMatchesTheMatlabSimulation(testCase)
            c = runSimulinkComparison("leadBraking", "automotive", 'SaveFigures', false);
            testCase.verifyLessThan(c.gapRmsError, 0.20, ...
                'Simulink and MATLAB gap trajectories disagree.');
            testCase.verifyLessThan(c.speedRmsError, 0.10, ...
                'Simulink and MATLAB speed trajectories disagree.');
        end

        function steadyFollowMatchesTheMatlabSimulation(testCase)
            % The undisturbed case should agree far more tightly than the
            % transient one: there is nothing for numerical differences to
            % amplify.
            c = runSimulinkComparison("steadyFollow", "automotive", 'SaveFigures', false);
            testCase.verifyLessThan(c.gapRmsError, 0.01);
            testCase.verifyLessThan(c.speedRmsError, 0.01);
        end
    end
end
