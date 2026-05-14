import argparse
import sys
from ultralytics import YOLO

def optimize_to_tensorrt(model_path):
    """
    Optimiza un modelo YOLO (.pt) al formato TensorRT (.engine).
    """
    try:
        print(f"🚀 Cargando el modelo: {model_path}")
        model = YOLO(model_path)

        print(f"⚙️  Iniciando exportación a TensorRT (FP16)...")
        # format="engine" -> TensorRT
        # device=0       -> Usa la primera GPU (imprescindible para .engine)
        # half=True      -> Activa precisión FP16 (más rápido)
        path_resultado = model.export(format="engine", device=0, half=True)
        
        print(f"✅ Éxito! Modelo optimizado guardado en: {path_resultado}")

    except Exception as e:
        print(f"❌ Error durante la optimización: {e}")
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Optimizador de modelos YOLO para GPUs NVIDIA usando TensorRT.")
    
    # Parámetro de entrada: ruta al modelo .pt
    parser.add_argument(
        "model", 
        type=str, 
        help="Ruta al archivo del modelo de PyTorch (ej: yolov8n-seg.pt o best.pt)"
    )

    args = parser.parse_args()

    # Ejecutar la optimización
    optimize_to_tensorrt(args.model)