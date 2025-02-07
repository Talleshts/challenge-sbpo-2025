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
import ilog.cplex.IloCplex;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
		solveWithCPLEX();
		return null;
	}

	public ChallengeSolution solveWithCPLEX() {
		try {
			IloCplex cplex = new IloCplex();

			// Definir variáveis de decisão
			IloIntVar[] orderVars = cplex.boolVarArray(orders.size());
			IloIntVar[] aisleVars = cplex.boolVarArray(aisles.size());

			// Definir função objetivo: maximizar a quantidade total de itens coletados
			IloLinearNumExpr objective = cplex.linearNumExpr();
			for (int i = 0; i < orders.size(); i++) {
				for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
					objective.addTerm(entry.getValue(), orderVars[i]);
				}
			}
			cplex.addMaximize(objective);

			// Definir restrições de capacidade dos corredores
			for (int j = 0; j < aisles.size(); j++) {
				IloLinearNumExpr aisleCapacity = cplex.linearNumExpr();
				for (Map.Entry<Integer, Integer> entry : aisles.get(j).entrySet()) {
					aisleCapacity.addTerm(entry.getValue(), aisleVars[j]);
				}
				cplex.addLe(aisleCapacity, waveSizeUB);
			}

			// Definir restrições de quantidade mínima e máxima de itens coletados
			IloLinearNumExpr totalItems = cplex.linearNumExpr();
			for (int i = 0; i < orders.size(); i++) {
				for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
					totalItems.addTerm(entry.getValue(), orderVars[i]);
				}
			}
			cplex.addGe(totalItems, waveSizeLB);
			cplex.addLe(totalItems, waveSizeUB);

			// Resolver o modelo
			if (cplex.solve()) {
				System.out.println("Solution status = " + cplex.getStatus());
				System.out.println("Solution value = " + cplex.getObjValue());

				// Criar e retornar a solução do desafio
				Set<Integer> selectedOrders = new HashSet<>();
				Set<Integer> visitedAisles = new HashSet<>();
				for (int i = 0; i < orders.size(); i++) {
					if (cplex.getValue(orderVars[i]) > 0.5) {
						selectedOrders.add(i);
					}
				}
				for (int j = 0; j < aisles.size(); j++) {
					if (cplex.getValue(aisleVars[j]) > 0.5) {
						visitedAisles.add(j);
					}
				}
				return new ChallengeSolution(selectedOrders, visitedAisles);
			} else {
				System.out.println("Solution not found");
				return null;
			}
		} catch (IloException e) {
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
