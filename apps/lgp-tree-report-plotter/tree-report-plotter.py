import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(description='Plot path tree optimization report.')
parser.add_argument('filepath', nargs=1, help='path to optimization report')

args = parser.parse_args()

if len(args.filepath) == 0:
  print("please provide filepath to path tree optimization report")

filepath = args.filepath[0]

file = open(filepath, "r")

### SEQUENTIAL PARSING###

stepsPerPhase_chunks = file.readline().split(" ")
assert stepsPerPhase_chunks[0] == "stepsPerPhase"
stepsPerPhase = int(stepsPerPhase_chunks[1])

totalCost_chunks = file.readline().split(" ")
assert totalCost_chunks[0] == "totalCost"
totalCost = float(totalCost_chunks[1])

qdim_chunks = file.readline().split(" ")
assert qdim_chunks[0] == "qdim"
qdim = int(qdim_chunks[1])

assert file.readline() == "\n"
assert file.readline() == "trajectoryTree\n"

# trajectory tree
trajectory_tree = []
q_chunks = file.readline().rstrip().split(" ")
while len(q_chunks) >= qdim + 1:
  q = []
  for chunk in q_chunks[1:]:
    q.append(float(chunk))
  trajectory_tree.append(q)
  q_chunks = file.readline().rstrip().split(" ")

assert q_chunks[0] == ''

# objectives
objectives = []
objective_chunks = file.readline().rstrip().split(" ")
while len(objective_chunks) == 3 and objective_chunks[0] == "objective":
  name = objective_chunks[1]
  type = objective_chunks[2]

  print("objective: {}, type: {}".format(name, type))

  objective_chunks = file.readline().rstrip().split(" ")
  ss = []
  costs = []
  while len(objective_chunks) == 2:
    ss.append(int(objective_chunks[0]))
    costs.append(float(objective_chunks[1]))

    objective_chunks = file.readline().rstrip().split(" ")

  objectives.append({
    "name": name,
    "type": type,
    "ss": ss,
    "costs": costs
  })

  assert objective_chunks[0] == ''
  objective_chunks = file.readline().rstrip().split(" ")

#print("objectives: {}".format(objectives))


# irrelevant objectives
irrelevant_objectives = []
irrelevant_objective_chunks = objective_chunks
assert objective_chunks[0] == "objectives_irrelevant_for_total_cost"
irrelevant_objective_chunks = file.readline().rstrip().split(" ")

while len(irrelevant_objective_chunks) == 1 and len(irrelevant_objective_chunks[0]) > 0:
  irrelevant_objectives.append(irrelevant_objective_chunks[0])
  irrelevant_objective_chunks = file.readline().rstrip().split(" ")

print("irrelevant_objectives: {}".format(irrelevant_objectives))

assert irrelevant_objective_chunks[0] == ''

# vars
var_chunks = file.readline().rstrip().split(" ")
vars = []
while len(var_chunks) == 2 and var_chunks[0] == "branch":
  id = var_chunks[1]
  var0 = []
  var_chunks = file.readline().rstrip().split(" ")

  while len(var_chunks) == 1 and len(var_chunks[0]) > 0:
    var0.append(int(var_chunks[0]))
    var_chunks = file.readline().rstrip().split(" ")

  vars.append(var0)
  var_chunks = file.readline().rstrip().split(" ")

#print("vars: {}".format(vars))

# plot costs branch by branch
fig, axs = plt.subplots(3, len(vars), sharex='col', sharey='row',)
lined = dict()

for i, var in enumerate(vars):
  all_lines_for_ax_0 = []
  ax_0 = axs[0][i]

  # objectives
  for objective in objectives:
    name = objective.get("name")
    s_to_cost = []

    # if "Transition" not in name:
    #   continue
    if name in irrelevant_objectives:
      continue

    for local, global_s in enumerate(var):
      # find slice in objective
      ss = objective.get("ss")
      costs = objective.get("costs")
      slice_index = list(filter(lambda i: (ss[i] == global_s), range(0, len(ss))))

      if len(slice_index) == 1:
        index = slice_index[0]
        s_to_cost.append((local, costs[index], name))

    line, = ax_0.plot([kv[0] for kv in s_to_cost], [kv[1] for kv in s_to_cost], '-x', label=name)
    all_lines_for_ax_0.append(line)
  leg_0 = ax_0.legend()

  # we will set up a dict mapping legend line to orig line, and enable
  # picking on the legend line
  for legline, origline in zip(leg_0.get_lines(), all_lines_for_ax_0):
      legline.set_picker(5)  # 5 pts tolerance
      lined[legline] = origline

  # q
  ax_1 = axs[1][i]
  all_lines_for_ax_1 = []
  for j in range(0, qdim):
    x=[]
    for local, global_s in enumerate(var):
      qi = trajectory_tree[global_s][j]
      x.append(qi)
    line, = ax_1.plot(x, '-', label="q.{}".format(j))
    all_lines_for_ax_1.append(line)
  leg_1 = ax_1.legend()

  # we will set up a dict mapping legend line to orig line, and enable
  # picking on the legend line
  for legline, origline in zip(leg_1.get_lines(), all_lines_for_ax_1):
      legline.set_picker(5)  # 5 pts tolerance
      lined[legline] = origline

  # for some addition plaots for journal paper only
  # q dot dot
  # def get_franka_scale(j):
  #   if j == 0 or j == 1:
  #     return 0.1
  #
  #   return 1.0
  #
  # def get_baxter_scale(j):
  #   return 1.0
  #
  # ax_2 = axs[2][i]
  # all_lines_for_ax_2 = []
  # for j in range(0, 1): #qdim):
  #   x = []
  #   for local, (global_sm2, global_sm1, global_s) in enumerate(zip(var, var[1:], var[2:])):
  #    qi_dot_dot = get_baxter_scale(j) * (2.0 * trajectory_tree[global_sm1][j] - trajectory_tree[global_sm2][j] - trajectory_tree[global_s][j])
  #    x.append(qi_dot_dot)
  #   line, = ax_2.plot(x, '-', label="q dot dot.{}".format(j))
  #   all_lines_for_ax_2.append(line)
  # leg_1 = ax_2.legend()
  # #ax_2.set_ylim(-0.1, 0.1)

  #we will set up a dict mapping legend line to orig line, and enable
  #picking on the legend line
  # for legline, origline in zip(leg_2.get_lines(), all_lines_for_ax_2):
  #  legline.set_picker(5)  # 5 pts tolerance
  #  lined[legline] = origline

  #for paper only (give q dot dot on branch 3)
  # if True:
  #  for j in range(0, qdim):
  #    scale = get_baxter_scale(j)
  #
  #    q_dot_dot = open("joint_{}_branch_{}_qddot.data".format(j, i), "w")
  #    for local, (global_sm2, global_sm1, global_s) in enumerate(zip(var, var[1:], var[2:])):
  #      qi_dot_dot =  scale * (2.0 * trajectory_tree[global_sm1][j] - trajectory_tree[global_sm2][j] - trajectory_tree[global_s][j]) * 40.0
  #      q_dot_dot.write("{}, {}\n".format(local+2, qi_dot_dot)) # order 2
  #
  #    q_dot = open("joint_{}_branch_{}_qdot.data".format(j, i), "w")
  #    for local, (global_sm1, global_s) in enumerate(zip(var, var[1:])):
  #      qi_dot = scale * (
  #                trajectory_tree[global_s][j] - trajectory_tree[global_sm1][j]) * 20.0
  #      q_dot.write("{}, {}\n".format(local + 1, qi_dot))  # order 2
  #
  #    q = open("joint_{}_branch_{}_q.data".format(j, i), "w")
  #    for local, global_s in enumerate(var):
  #      qi = scale * trajectory_tree[global_s][j]
  #      q.write("{}, {}\n".format(local, qi)) # order 2


def onpick(event):
  # on the pick event, find the orig line corresponding to the
  # legend proxy line, and toggle the visibility
  legline = event.artist
  origline = lined[legline]

  vis = not origline.get_visible()
  origline.set_visible(vis)
  # Change the alpha on the line in the legend so we can see what lines
  # have been toggled
  if vis:
    legline.set_alpha(1.0)
  else:
    legline.set_alpha(0.2)
  fig.canvas.draw()

fig.canvas.mpl_connect('pick_event', onpick)

plt.show()