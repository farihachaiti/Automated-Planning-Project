import sys
import re

def parse_plan_file(plan_file):
    """
    Parses the plan file and returns the final state as a list of predicates.
    """
    final_state = []

    with open(plan_file, 'r') as file:
        for line in file:
            action = line.strip().lower()
            # Match and process actions
            if "move" in action:
                match = re.match(r'\(move (\w+) (\w+) (\w+) (\w+) (\w+)\)', action)
                if match:
                    content, robot, from_loc, to_loc, box = match.groups()
                    final_state.append(f"(atl {robot} {to_loc})")
                    final_state.append(f"(belong {box} {to_loc})")
            elif "fillbox" in action:
                match = re.match(r'\(fillbox (\w+) (\w+) (\w+) (\w+)\)', action)
                if match:
                    box, robot, loc, content = match.groups()
                    final_state.append(f"(contain {box} {content})")
                    final_state.append(f"(left {robot} {content})")
                    final_state.append(f"(not (holding {robot} {content}))")
            elif "emptybox" in action:
                match = re.match(r'\(emptybox (\w+) (\w+) (\w+) (\w+) (\w+)\)', action)
                if match:
                    robot, box, content, loc, workstation = match.groups()
                    final_state.append(f"(not (contain {box} {content}))")
                    final_state.append(f"(empty {box})")
                    final_state.append(f"(unloaded {robot})")
                    final_state.append(f"(attached {content} {loc})")
                    final_state.append(f"(attachedtoWS {content} {workstation})")
            elif "pickupbox" in action:
                match = re.match(r'\(pickupbox (\w+) (\w+) (\w+)\)', action)
                if match:
                    robot, box, loc = match.groups()
                    final_state.append(f"(loaded {robot} {box})")
            elif "deliver" in action:
                match = re.match(r'\(deliver (\w+) (\w+) (\w+) (\w+) (\w+) (\w+)\)', action)
                if match:
                    robot, box, from_loc, to_loc, from_ws, to_ws = match.groups()
                    final_state.append(f"(has {to_ws} {box})")
                    final_state.append(f"(belong {box} {to_loc})")
                    final_state.append(f"(unloaded {robot})")
                    final_state.append(f"(atl {robot} {to_loc})")

    return final_state

def update_problem_file(final_state, problem_file):
    """
    Updates the initial state in the problem file with the final state from the plan.
    """
    with open(problem_file, 'r') as file:
        lines = file.readlines()

    updated_lines = []
    in_init = False

    for line in lines:
        if "(:init" in line:
            in_init = True
            updated_lines.append(line)
            for state in final_state:
                updated_lines.append(f"    {state}\n")
            continue

        if in_init:
            if ")" in line:
                in_init = False
                updated_lines.append(line)
            continue

        updated_lines.append(line)

    with open(problem_file, 'w') as file:
        file.writelines(updated_lines)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python update_problem.py <plan_file> <problem_file>")
        sys.exit(1)

    plan_file = sys.argv[1]
    problem_file = sys.argv[2]

    final_state = parse_plan_file(plan_file)
    update_problem_file(final_state, problem_file)
    print(f"Updated {problem_file} with the final state from {plan_file}")
