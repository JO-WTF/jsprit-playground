package com.graphhopper.playground;

import java.io.IOException;
import java.util.Collection;

import com.graphhopper.jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import com.graphhopper.jsprit.analysis.toolbox.Plotter;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.box.Jsprit;
import com.graphhopper.jsprit.core.algorithm.state.StateId;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.algorithm.state.StateUpdater;
import com.graphhopper.jsprit.core.algorithm.termination.VariationCoefficientTermination;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.constraint.ConstraintManager;
import com.graphhopper.jsprit.core.problem.constraint.SoftActivityConstraint;
import com.graphhopper.jsprit.core.problem.cost.ForwardTransportTime;
import com.graphhopper.jsprit.core.problem.cost.VehicleRoutingActivityCosts;
import com.graphhopper.jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import com.graphhopper.jsprit.io.problem.VrpXMLReader;
import com.graphhopper.jsprit.core.problem.misc.JobInsertionContext;
import com.graphhopper.jsprit.core.problem.solution.SolutionCostCalculator;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.solution.route.VehicleRoute;
import com.graphhopper.jsprit.core.problem.solution.route.activity.ActivityVisitor;
import com.graphhopper.jsprit.core.problem.solution.route.activity.End;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TourActivity;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter.Print;
import com.graphhopper.jsprit.core.util.ActivityTimeTracker;
import com.graphhopper.jsprit.core.util.Solutions;
import com.graphhopper.jsprit.core.util.VehicleRoutingTransportCostsMatrix;
import com.graphhopper.playground.util.MatrixReader;

public class AbeMinMax {
    /*
     * This updates the state "max-transport-time" which is introduced below. Once
     * either the insertion procedure starts or a job has been inserted,
     * UpdateMaxTransportTime is called for the route that has been changed.
     *
     * It must not only be an ActivityVisitor which indicates that the update
     * procedure starts at the beginning of route all the way to end (in contrary to
     * the ReverseActivityVisitor) but also be a StateUpdater which is just a marker
     * to register it in the StateManager.
     *
     * You do not need to declare this as static inner class. You can just choose
     * your preferred approach. However, be aware that this requires the stateName
     * "max-transport-time" you define below. If you choose to define this as class
     * in a new file, you might define "max-transport-time" as static id in another
     * file, to make sure you do not have type errors etc..
     */
    static class UpdateMaxTransportTime implements ActivityVisitor, StateUpdater {

        private StateManager stateManager;

        private ActivityTimeTracker timeTracker;

        public UpdateMaxTransportTime(StateManager stateManager, ForwardTransportTime transportTime,
                VehicleRoutingActivityCosts activityCosts) {
            super();
            this.stateManager = stateManager;
            this.timeTracker = new ActivityTimeTracker(transportTime, activityCosts);
        }

        @Override
        public void begin(VehicleRoute route) {
            timeTracker.begin(route);
        }

        @Override
        public void visit(TourActivity activity) {
            timeTracker.visit(activity);
        }

        @Override
        public void finish() {
            timeTracker.finish();
            double newRouteEndTime = timeTracker.getActArrTime();
            Double maxTimeObject = stateManager.getProblemState(stateManager.createStateId("max-transport-time"),
                    Double.class);
            double currentMaxTransportTime;
            if (maxTimeObject == null || maxTimeObject.isNaN()) {
                currentMaxTransportTime = 0.0;
            } else {
                currentMaxTransportTime = maxTimeObject.doubleValue();
            }
            if (newRouteEndTime > currentMaxTransportTime) {
                stateManager.putProblemState(stateManager.createStateId("max-transport-time"), Double.class,
                        newRouteEndTime);
            }
        }

    }

    static class PenalizeShiftOfMaxTransportTime implements SoftActivityConstraint {
        private VehicleRoutingTransportCosts routingCosts;
        private VehicleRoutingActivityCosts activityCosts;
        private StateManager stateManager;

        private PenalizeShiftOfMaxTransportTime(VehicleRoutingProblem problem, StateManager stateManager) {
            routingCosts = problem.getTransportCosts();
            activityCosts = problem.getActivityCosts();
            this.stateManager = stateManager;
        }

        @Override
        public double getCosts(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct,
                TourActivity nextAct, double depTimeAtPrevAct) {
            /*
             * determines maximum of all routes' transport times, which is here basically a
             * state that can be fetched via the stateManager
             */
            final double penaltyForEachTimeUnitAboveCurrentMaxTime = 3.;
            Double maxTimeObject = stateManager.getProblemState(stateManager.createStateId("max-transport-time"),
                    Double.class);
            double currentMaxTransportTime;
            if (maxTimeObject == null || maxTimeObject.isNaN()) {
                currentMaxTransportTime = 0.0;
            } else {
                currentMaxTransportTime = maxTimeObject;
            }

            double tp_costs_prevAct_newAct = routingCosts.getTransportCost(prevAct.getLocation(), newAct.getLocation(),
                    depTimeAtPrevAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
            double tp_time_prevAct_newAct = routingCosts.getTransportTime(prevAct.getLocation(), newAct.getLocation(),
                    depTimeAtPrevAct, iFacts.getNewDriver(), iFacts.getNewVehicle());

            double newAct_arrTime = depTimeAtPrevAct + tp_time_prevAct_newAct;
            double newAct_endTime = Math.max(newAct_arrTime, newAct.getTheoreticalEarliestOperationStartTime())
                    + activityCosts.getActivityDuration(newAct, newAct_arrTime, iFacts.getNewDriver(),
                            iFacts.getNewVehicle());

            // open routes
            if (nextAct instanceof End) {
                if (!iFacts.getNewVehicle().isReturnToDepot()) {
                    double new_routes_transport_time = iFacts.getRoute().getEnd().getArrTime()
                            - iFacts.getRoute().getStart().getEndTime() + tp_time_prevAct_newAct;
                    return tp_costs_prevAct_newAct + penaltyForEachTimeUnitAboveCurrentMaxTime
                            * Math.max(0, new_routes_transport_time - currentMaxTransportTime);
                }
            }

            double tp_costs_newAct_nextAct = routingCosts.getTransportCost(newAct.getLocation(), nextAct.getLocation(),
                    newAct_endTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
            double tp_time_newAct_nextAct = routingCosts.getTransportTime(newAct.getLocation(), nextAct.getLocation(),
                    newAct_endTime, iFacts.getNewDriver(), iFacts.getNewVehicle());

            double totalCosts = tp_costs_prevAct_newAct + tp_costs_newAct_nextAct;

            double oldCosts;
            if (iFacts.getRoute().isEmpty()) {
                oldCosts = routingCosts.getTransportCost(prevAct.getLocation(), nextAct.getLocation(), depTimeAtPrevAct,
                        iFacts.getNewDriver(), iFacts.getNewVehicle());
            } else {
                oldCosts = routingCosts.getTransportCost(prevAct.getLocation(), nextAct.getLocation(),
                        prevAct.getEndTime(), iFacts.getRoute().getDriver(), iFacts.getRoute().getVehicle());
            }

            double nextAct_arrTime = newAct_endTime + tp_time_newAct_nextAct;
            double oldTime;
            if (iFacts.getRoute().isEmpty()) {
                oldTime = (nextAct.getArrTime() - depTimeAtPrevAct);
            } else {
                oldTime = (nextAct.getArrTime() - iFacts.getRoute().getDepartureTime());
            }

            double additionalTime = (nextAct_arrTime - iFacts.getNewDepTime()) - oldTime;
            double tpTime = iFacts.getRoute().getEnd().getArrTime() - iFacts.getRoute().getStart().getEndTime()
                    + additionalTime;

            return totalCosts - oldCosts
                    + penaltyForEachTimeUnitAboveCurrentMaxTime * Math.max(0, tpTime - currentMaxTransportTime);

        }
    }

    static class ObjectiveFunction implements SolutionCostCalculator {

        @Override
        public double getCosts(VehicleRoutingProblemSolution solution) {
            double scalingParameter = 0.2;
            double maxTransportTime = 0.;
            double sumTransportTimes = 0.;
            for (VehicleRoute route : solution.getRoutes()) {
                double tpTime = route.getEnd().getArrTime() - route.getStart().getEndTime();
                sumTransportTimes += tpTime;
                if (tpTime > maxTransportTime) {
                    maxTransportTime = tpTime;
                }
            }
            return maxTransportTime + scalingParameter * sumTransportTimes;
        }
    }

    public static void main(String[] args) throws IOException {

        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
        new VrpXMLReader(vrpBuilder).read("input/abeProblem.xml");
        VehicleRoutingTransportCostsMatrix.Builder matrixBuilder = VehicleRoutingTransportCostsMatrix.Builder
                .newInstance(true);
        final MatrixReader matrixReader = new MatrixReader(matrixBuilder);
        matrixReader.read("input/abeProblemMatrix.txt");
        VehicleRoutingTransportCostsMatrix matrix = matrixBuilder.build();
        vrpBuilder.setRoutingCost(matrix);
        final VehicleRoutingProblem problem = vrpBuilder.build();

        /*
         * Your custom objective function that min max transport times. Additionally you
         * can try to consider overall transport times in your objective as well. Thus
         * you minimize max transport times first, and second, you minimize overall
         * transport time.
         *
         * If you choose to consider overall transport times, makes sure you scale it
         * appropriately.
         */
        StateManager stateManager = new StateManager(problem);
        // introduce a new state called "max-transport-time"
        StateId max_transport_time_state = stateManager.createStateId("max-transport-time");
        // add a default-state for "max-transport-time"
        stateManager.putProblemState(max_transport_time_state, Double.class, 0.);
        //
        stateManager.addStateUpdater(
                new UpdateMaxTransportTime(stateManager, problem.getTransportCosts(), problem.getActivityCosts()));

        /*
         * The insertion heuristics is controlled with your constraints
         */
        ConstraintManager constraintManager = new ConstraintManager(problem, stateManager);
        /*
         * soft constraint that penalizes a shift of max-route transport time, i.e. once
         * the insertion heuristic tries to insert a jobActivity at position which
         * results in a shift of max-transport-time, it is penalized with
         * penaltyForEachTimeUnitAboveCurrentMaxTime
         *
         */
        constraintManager.addConstraint(new PenalizeShiftOfMaxTransportTime(problem, stateManager));

        VehicleRoutingAlgorithm vra = Jsprit.Builder.newInstance(problem)
                .setStateAndConstraintManager(stateManager, constraintManager)
                .setObjectiveFunction(new ObjectiveFunction()).buildAlgorithm();

        vra.setMaxIterations(2000);

        vra.addListener(new AlgorithmSearchProgressChartListener("output/abeMinMaxProblemProgress.png"));
        VariationCoefficientTermination prematureAlgorithmTermination = new VariationCoefficientTermination(150, 0.001);
        vra.addListener(prematureAlgorithmTermination);
        vra.setPrematureAlgorithmTermination(prematureAlgorithmTermination);

        Collection<VehicleRoutingProblemSolution> solutions = vra.searchSolutions();

        Plotter plotter2 = new Plotter(problem, Solutions.bestOf(solutions)).setLabel(Plotter.Label.ID);
        plotter2.plot("output/abeMinMaxProblemWithSolution.png", "abe");

        SolutionPrinter.print(problem, Solutions.bestOf(solutions), Print.VERBOSE);

        System.out.println("total-time: " + getTotalTime(Solutions.bestOf(solutions)));
        System.out.println("total-distance: " + getTotalDistance(matrixReader, Solutions.bestOf(solutions)));

    }

    private static double getTotalDistance(MatrixReader matrix, VehicleRoutingProblemSolution bestOf) {
        double dist = 0.0;
        for (VehicleRoute r : bestOf.getRoutes()) {
            TourActivity last = r.getStart();
            for (TourActivity act : r.getActivities()) {
                dist += matrix.getDistance(last.getLocation().getId(), act.getLocation().getId());
                last = act;
            }
            dist += matrix.getDistance(last.getLocation().getId(), r.getEnd().getLocation().getId());
        }
        return dist;
    }

    private static double getTotalTime(VehicleRoutingProblemSolution bestOf) {
        double time = 0.0;
        for (VehicleRoute r : bestOf.getRoutes())
            time += r.getEnd().getArrTime();
        return time;
    }

}