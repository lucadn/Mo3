import numpy as np


def generate_binding_matrix(params):

    n_nodes = params["node"]["number_of_nodes"]

    bm_params = params["mobility"]["binding_matrix"]

    if not bm_params["auto_generate"]:
        return

    switches = bm_params["switches"]

    filename = params["mobility"]["BMfilename"]

    with open(filename, "w") as f:

        for state in switches:

            matrix_type = state["type"]

            # Generate the matrix
            if matrix_type == "all_together":
                BM = np.ones((n_nodes, n_nodes), dtype=int)

            elif matrix_type == "independent":
                BM = np.eye(n_nodes, dtype=int)

            else:
                raise ValueError(
                    f"Unknown binding matrix type: {matrix_type}"
                )

            # Write the matrix
            for row in BM:
                f.write(" ".join(map(str, row)))
                f.write("\n")

            # Write nextSwitch if present
            if "nextSwitch" in state:
                f.write(f"nextSwitch {state['nextSwitch']}\n")