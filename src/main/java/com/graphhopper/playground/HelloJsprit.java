package com.graphhopper.playground;

import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.box.Jsprit;
import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.job.Service;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.util.Solutions;
import com.graphhopper.jsprit.io.problem.VrpXMLWriter;
import com.graphhopper.playground.util.ioUtils;

import java.util.Collection;

public class HelloJsprit {
    public static void main(String[] args) {
        /*
         * some preparation - create output folder
         */
        ioUtils.createOutputFolder();

        /*
         * Define a builder to build the VehicleRoutingProblem
         */
        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();

        /*
         * Define a builder to build vehicles
         */
        VehicleImpl.Builder vehicleBuilder = VehicleImpl.Builder.newInstance("vehicle");
        vehicleBuilder.setStartLocation(Location.newInstance(10, 10));
        // build vehicle
        VehicleImpl vehicle = vehicleBuilder.build();

        /*
         * build services with id 1...4 at the required locations
         */
        Service service1 = Service.Builder.newInstance("service1").setLocation(Location.newInstance(5, 7)).build();
        Service service2 = Service.Builder.newInstance("service2").setLocation(Location.newInstance(5, 13)).build();
        Service service3 = Service.Builder.newInstance("service3").setLocation(Location.newInstance(15, 7)).build();
        Service service4 = Service.Builder.newInstance("service4").setLocation(Location.newInstance(15, 13)).build();

        /*
         * Add vehicles and services to the problem builder
         */
        vrpBuilder.addVehicle(vehicle);
        vrpBuilder.addJob(service1).addJob(service2).addJob(service3).addJob(service4);

        /*
         * Build the problem.
         */
        VehicleRoutingProblem problem = vrpBuilder.build();

        /*
         * Setup algorithm
         */
        VehicleRoutingAlgorithm algorithm = Jsprit.createAlgorithm(problem);

        /*
         * Search solutions of the problem and write them to file
         */
        Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();
        new VrpXMLWriter(problem, solutions).write("output/HelloJspritProblemWithSolution.xml");

        /*
         * Get the best solution and print it
         */
        VehicleRoutingProblemSolution bestSolution = Solutions.bestOf(solutions);
        SolutionPrinter.print(problem, bestSolution, SolutionPrinter.Print.VERBOSE);

        /*
         * Graph the problem and best solution
         */
        new GraphStreamViewer(problem, bestSolution).labelWith(GraphStreamViewer.Label.ID).display();
    }
}
