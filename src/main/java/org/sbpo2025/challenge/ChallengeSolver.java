package org.sbpo2025.challenge;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import org.apache.commons.lang3.time.StopWatch;

import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class ChallengeSolver {
	private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

	protected List<Map<Integer, Integer>> orders;
	protected List<Map<Integer, Integer>> aisles;
	protected int nItems;
	protected int waveSizeLB;
	protected int waveSizeUB;

	public ChallengeSolver(
			List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB,
			int waveSizeUB) {
		this.orders = orders;
		this.aisles = aisles;
		this.nItems = nItems;
		this.waveSizeLB = waveSizeLB;
		this.waveSizeUB = waveSizeUB;
	}

	public ChallengeSolution solve(StopWatch stopWatch) {
		return solveWithCPLEX(stopWatch);
	}

	public ChallengeSolution solveWithCPLEX(StopWatch stopWatch) {
		try (IloCplex cplex = new IloCplex()) {
			cplex.setParam(IloCplex.Param.TimeLimit, 540);
			cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.000000000000001);
			cplex.setParam(IloCplex.Param.Threads, 0);

			IloIntVar[] orderVars = cplex.boolVarArray(orders.size());
			IloIntVar[] aisleVars = cplex.boolVarArray(aisles.size());

			IloIntVar N = cplex.intVar(waveSizeLB, waveSizeUB, "N");
			IloIntVar D = cplex.intVar(1, aisles.size(), "D");
			IloNumVar Z = cplex.numVar(0, Double.MAX_VALUE, "Z");

			for (int i = 0; i < nItems; i++) {
				IloLinearNumExpr itemBalance = cplex.linearNumExpr();

				for (int p = 0; p < orders.size(); p++) {
					if (orders.get(p).containsKey(i)) {
						itemBalance.addTerm(orders.get(p).get(i), orderVars[p]);
					}
				}

				for (int c = 0; c < aisles.size(); c++) {
					if (aisles.get(c).containsKey(i)) {
						itemBalance.addTerm(-aisles.get(c).get(i), aisleVars[c]);
					}
				}

				cplex.addLe(itemBalance, 0);
			}

			IloLinearNumExpr totalAisles = cplex.linearNumExpr();
			for (int j = 0; j < aisles.size(); j++) {
				totalAisles.addTerm(1, aisleVars[j]);
			}
			cplex.addEq(totalAisles, D);

			IloLinearNumExpr totalItems = cplex.linearNumExpr();
			for (int i = 0; i < orders.size(); i++) {
				int orderTotal = orders.get(i).values().stream().mapToInt(Integer::intValue).sum();
				totalItems.addTerm(orderTotal, orderVars[i]);
			}
			cplex.addEq(totalItems, N);

			IloLinearNumExpr linearization = cplex.linearNumExpr();
			linearization.addTerm(1, Z);
			linearization.addTerm(-1, N);
			linearization.addTerm(waveSizeUB, D);
			cplex.addLe(linearization, waveSizeUB * aisles.size());

			cplex.addMaximize(Z);

			if (cplex.solve()) {
				Set<Integer> selectedOrders = new HashSet<>();
				Set<Integer> selectedAisles = new HashSet<>();

				for (int i = 0; i < orders.size(); i++) {
					if (cplex.getValue(orderVars[i]) > 0.5) {
						selectedOrders.add(i);
					}
				}

				for (int j = 0; j < aisles.size(); j++) {
					if (cplex.getValue(aisleVars[j]) > 0.5) {
						selectedAisles.add(j);
					}
				}

				ChallengeSolution solution = new ChallengeSolution(selectedOrders, selectedAisles);

				if (isSolutionFeasible(solution)) {
					System.out.println("Tempo restante: " + getRemainingTime(stopWatch) + " segundos");
					System.out.println("Valor da funcao objetivo: " + computeObjectiveFunction(solution));

					return solution;
				} else {
					return null;
				}
			} else {
				System.out.println("Solução não encontrada");
				return null;
			}
		} catch (IloException e) {
			System.err.println("Erro ao resolver o modelo CPLEX: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}

	/*
	 * Get the remaining time in seconds
	 */
	protected long getRemainingTime(StopWatch stopWatch) {
		return Math.max(
				TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
				0);
	}

	protected boolean isSolutionFeasible(ChallengeSolution challengeSolution) {
		Set<Integer> selectedOrders = challengeSolution.orders();
		Set<Integer> visitedAisles = challengeSolution.aisles();
		if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
			return false;
		}

		int[] totalUnitsPicked = new int[nItems];
		int[] totalUnitsAvailable = new int[nItems];

		// Calculate total units picked
		for (int order : selectedOrders) {
			for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
				totalUnitsPicked[entry.getKey()] += entry.getValue();
			}
		}

		// Calculate total units available
		for (int aisle : visitedAisles) {
			for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
				totalUnitsAvailable[entry.getKey()] += entry.getValue();
			}
		}

		// Check if the total units picked are within bounds
		int totalUnits = Arrays.stream(totalUnitsPicked).sum();
		if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
			return false;
		}

		// Check if the units picked do not exceed the units available
		for (int i = 0; i < nItems; i++) {
			if (totalUnitsPicked[i] > totalUnitsAvailable[i]) {
				return false;
			}
		}

		return true;
	}

	protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
		Set<Integer> selectedOrders = challengeSolution.orders();
		Set<Integer> visitedAisles = challengeSolution.aisles();
		if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
			return 0.0;
		}
		int totalUnitsPicked = 0;

		// Calculate total units picked
		for (int order : selectedOrders) {
			totalUnitsPicked += orders.get(order).values().stream()
					.mapToInt(Integer::intValue)
					.sum();
		}

		// Calculate the number of visited aisles
		int numVisitedAisles = visitedAisles.size();

		// Objective function: total units picked / number of visited aisles
		return (double) totalUnitsPicked / numVisitedAisles;
	}

}