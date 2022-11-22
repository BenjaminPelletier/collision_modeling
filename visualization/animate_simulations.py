import discrete_sampling_model
import reich_model
from viz import EncounterVisualizer, MotionModel


models = [
    MotionModel(
        name="Discrete sampling model on parallel paths",
        make_flights=discrete_sampling_model.make_parallel_paths
    ),
    MotionModel(
        name="Reich model on parallel paths",
        make_flights=reich_model.make_parallel_paths
    ),
]
app = EncounterVisualizer(models)
app.run()
