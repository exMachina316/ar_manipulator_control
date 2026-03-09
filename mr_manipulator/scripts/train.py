import pandas as pd
import numpy as np
import pickle
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.svm import SVC
import xgboost as xgb


# ==================== CONFIGURATION ====================
DATA_FILE = 'new_3_gesture_data.csv'
model_filename = 'xgboost_gesture_model.p'
TEST_SIZE = 0.2
RANDOM_STATE = 42


def get_model():
    """XGBoost"""
    return xgb.XGBClassifier(
        n_estimators=100,
        max_depth=6,
        learning_rate=0.1,
        objective='multi:softmax',
        random_state=RANDOM_STATE,
        eval_metric='mlogloss'
    )


# ==================== DATA LOADING ====================
def load_data(filepath):
    """Load and prepare the dataset"""
    print(f"Loading data from {filepath}...")
    df = pd.read_csv(filepath)
    
    # Separate features and labels
    X = df.iloc[:, 1:].values  # All columns except first (class label)
    y = df.iloc[:, 0].values   # First column (class label)
    
    print(f"Dataset shape: {X.shape}")
    print(f"Number of classes: {len(np.unique(y))}")
    print(f"Class distribution: {np.bincount(y.astype(int))}")
    
    return X, y


# ==================== VISUALIZATION ====================
def plot_confusion_matrix(y_true, y_pred, class_names=None):
    """Plot confusion matrix with better visualization"""
    cm = confusion_matrix(y_true, y_pred)
    
    plt.figure(figsize=(10, 8))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                xticklabels=class_names, yticklabels=class_names,
                cbar_kws={'label': 'Count'})
    plt.title('Confusion Matrix', fontsize=16, fontweight='bold')
    plt.ylabel('True Label', fontsize=12)
    plt.xlabel('Predicted Label', fontsize=12)
    plt.tight_layout()
    plt.show()


def print_classification_report(y_true, y_pred, class_names=None):
    """Print detailed classification report"""
    print("\n" + "="*60)
    print("CLASSIFICATION REPORT")
    print("="*60)
    report = classification_report(y_true, y_pred, 
                                   target_names=class_names,
                                   digits=4)
    print(report)
    print("="*60 + "\n")


# ==================== TRAINING ====================
def train_model():
    """
    Main training function
    
    Args:
        model_name: Name of the model to train ('xgboost', 'random_forest', 'svm')
    """
    # Load data
    X, y = load_data(DATA_FILE)
    
    # Train-test split
    print(f"\nSplitting data: {int((1-TEST_SIZE)*100)}% train, {int(TEST_SIZE*100)}% test")
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=TEST_SIZE, random_state=RANDOM_STATE, stratify=y
    )
    
    print(f"Training samples: {len(X_train)}")
    print(f"Testing samples: {len(X_test)}")
    
    print(f"\n{'='*60}")
    print(f"Training xgboost model...")
    print(f"{'='*60}")
    
    model = get_model()
    
    # Train
    model.fit(X_train, y_train)
    print("✓ Training completed!")
    
    # Evaluate on training set
    y_train_pred = model.predict(X_train)
    train_accuracy = accuracy_score(y_train, y_train_pred)
    print(f"\nTraining Accuracy: {train_accuracy:.4f} ({train_accuracy*100:.2f}%)")
    
    # Evaluate on test set
    y_test_pred = model.predict(X_test)
    test_accuracy = accuracy_score(y_test, y_test_pred)
    print(f"Test Accuracy: {test_accuracy:.4f} ({test_accuracy*100:.2f}%)")
    
    # Generate class names
    class_names = [f"Gesture {i}" for i in range(len(np.unique(y)))]
    
    # Visualization
    print_classification_report(y_test, y_test_pred, class_names)
    plot_confusion_matrix(y_test, y_test_pred, class_names)
    
    # Save model
    with open(model_filename, 'wb') as f:
        pickle.dump(model, f)
    print(f"\n✓ Model saved as '{model_filename}'")
    
    return model, test_accuracy


# ==================== MAIN ====================
if __name__ == "__main__":
    print("\n" + "="*60)
    print("HAND GESTURE RECOGNITION - MODEL TRAINING")
    print("="*60 + "\n")
    
    trained_model, accuracy = train_model()
    
    print("\n" + "="*60)
    print("TRAINING COMPLETE!")
    print(f"Final Test Accuracy: {accuracy*100:.2f}%")