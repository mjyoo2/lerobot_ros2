"""
Training configuration schemas.
"""

from dataclasses import dataclass, field
from typing import Dict, Any, Optional


@dataclass
class TrainingConfig:
    """
    Training configuration for robot learning policies.

    Compatible with LeRobot's training scripts.
    """

    # Dataset
    dataset_repo_id: str  # HuggingFace dataset repository
    dataset_local_path: Optional[str] = None

    # Policy
    policy_type: str = "act"  # "act", "diffusion", "vqbet", etc.
    policy_name: Optional[str] = None

    # Training parameters
    num_epochs: int = 5000
    batch_size: int = 8
    learning_rate: float = 1e-4
    weight_decay: float = 1e-4

    # Device
    device: str = "cuda"  # "cuda" or "cpu"
    num_workers: int = 4

    # Checkpointing
    output_dir: str = "outputs/train"
    save_freq: int = 500  # Save every N epochs
    eval_freq: int = 100  # Evaluate every N epochs

    # Logging
    log_freq: int = 10  # Log every N steps
    wandb_enabled: bool = False
    wandb_project: Optional[str] = None
    wandb_entity: Optional[str] = None

    # Additional policy-specific parameters
    policy_params: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for use with LeRobot training scripts."""
        return {
            "dataset": {
                "repo_id": self.dataset_repo_id,
                "local_path": self.dataset_local_path,
            },
            "policy": {
                "type": self.policy_type,
                "name": self.policy_name or f"{self.policy_type}_policy",
                **self.policy_params,
            },
            "training": {
                "num_epochs": self.num_epochs,
                "batch_size": self.batch_size,
                "learning_rate": self.learning_rate,
                "weight_decay": self.weight_decay,
                "device": self.device,
                "num_workers": self.num_workers,
            },
            "logging": {
                "output_dir": self.output_dir,
                "log_freq": self.log_freq,
                "save_freq": self.save_freq,
                "eval_freq": self.eval_freq,
                "wandb": {
                    "enabled": self.wandb_enabled,
                    "project": self.wandb_project,
                    "entity": self.wandb_entity,
                },
            },
        }
