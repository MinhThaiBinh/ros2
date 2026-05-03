import random
import copy
import csv
import os
import numpy as np

class Individual:
    def __init__(self, genome_length):
        self.genome = [random.uniform(-1, 1) for _ in range(genome_length)]
        self.fitness = -1000.0 

class GeneticAlgorithm:
    def __init__(self, population_size, genome_length):
        self.population_size = population_size
        self.genome_length = genome_length
        self.population = [Individual(genome_length) for _ in range(population_size)]
        self.current_idx = 0
        self.generation = 1
        self.best_ever_fitness = -1000.0
        self.pkg_dir = "/home/congminh/ros2_ws/src/mecanum_ga_pkg"
        self.log_file = os.path.join(self.pkg_dir, "logs", "training_history.csv")
        self.model_dir = os.path.join(self.pkg_dir, "saved_models")
        if not os.path.exists(self.log_file):
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['generation', 'best_fitness', 'avg_fitness', 'min_fitness'])
    
    def get_next_genome(self):
        return self.population[self.current_idx].genome
    
    def save_fitness(self, score):
        self.population[self.current_idx].fitness = score
        self.current_idx += 1
        if self.current_idx >= self.population_size:
            self.evolve()
            self.current_idx = 0
            self.generation += 1

    def evolve(self):
        fitness_values = [ind.fitness for ind in self.population]
        best_f = max(fitness_values)
        avg_f = sum(fitness_values) / len(fitness_values)
        min_f = min(fitness_values)
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.generation, best_f, avg_f, min_f])
        self.population.sort(key=lambda ind: ind.fitness, reverse=True)
        best_ind = self.population[0]
        print(f"\n==================================================")
        print(f"--- TIEN HOA THE HE {self.generation} ---")
        print(f"Best: {best_f:.2f} | Avg: {avg_f:.2f}")
        os.makedirs(self.model_dir, exist_ok=True)
        np.save(os.path.join(self.model_dir, f"checkpoint_gen_{self.generation}.npy"), np.array(best_ind.genome))
        if best_f > self.best_ever_fitness:
            self.best_ever_fitness = best_f
            np.save(os.path.join(self.model_dir, "best_model_ever.npy"), np.array(best_ind.genome))
            print(f"*** KY LUC MOI! Da luu best_model_ever.npy ***")
        print(f"==================================================\n")
        new_population = [copy.deepcopy(self.population[0]), copy.deepcopy(self.population[1])]
        top_performers = self.population[:max(2, self.population_size // 2)]
        while len(new_population) < self.population_size:
            p1 = random.choice(top_performers)
            p2 = random.choice(top_performers)
            child = Individual(self.genome_length)
            child.genome = [random.choice([a, b]) for a, b in zip(p1.genome, p2.genome)]
            new_population.append(child)
        self.population = new_population
