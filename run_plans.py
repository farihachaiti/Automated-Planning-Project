import subprocess
import sys

def run_planner(domain_file, problem_file, plan_file):
    """
    Runs the planner with the given domain and problem files, and saves the output to plan_file.
    """
    planner_command = f'planner {domain_file} {problem_file} --search "astar(lmcut())" > {plan_file}'
    result = subprocess.run(planner_command, shell=True)
    if result.returncode != 0:
        print(f"Error running planner for {domain_file} and {problem_file}.")
        sys.exit(1)

def update_problem(plan_file, problem_file):
    """
    Runs the update_problem.py script to update the problem file with the final state from the plan file.
    """
    update_command = f'python update_plan.py --input {plan_file} --output {problem_file}'
    result = subprocess.run(update_command, shell=True)
    if result.returncode != 0:
        print(f"Error updating problem file {problem_file} with plan {plan_file}.")
        sys.exit(1)

def main():
    # Step 1: Run the first problem
    run_planner('/domain.pddl', '/problem1.pddl', 'plan1.txt')

    # Step 2: Update initial state of problem 2 based on the output of problem 1
    update_problem('plan1.txt', '/problem2.pddl')
    run_planner('/domain.pddl', '/problem2.pddl', 'plan2.txt')

    # Step 4: Repeat the process for subproblem 3, 4, and 5 (you can extend this as needed)
    # Example for subproblem 3
    update_problem('plan2.txt', '/problem3.pddl')
    run_planner('/domain.pddl', '/problem3.pddl', 'plan3.txt')

    # Example for subproblem 4
    update_problem('plan2.txt', '/problem4.pddl')
    run_planner('/domain.pddl', '/problem4.pddl', 'plan4.txt')

    # Example for subproblem 5
    update_problem('plan4.txt', '/problem5.pddl')
    run_planner('/domain.pddl', '/problem5.pddl', 'plan5.txt')

if __name__ == "__main__":
    main()
