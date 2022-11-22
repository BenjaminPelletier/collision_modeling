import discrete_sampling_model
from viz import EncounterVisualizer


app = EncounterVisualizer(discrete_sampling_model.make_parallel_paths_opposite_direction)
app.run()
