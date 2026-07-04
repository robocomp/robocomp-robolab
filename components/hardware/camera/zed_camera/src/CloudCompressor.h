#include <zstd.h>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <Lidar3D.h> // Asumo definiciones como TShortArray (vector<int16_t>) y TByteArray (vector<int8_t>)
#include <variant>
#include <omp.h>
#include <array>
#include <cstring> // Para memcpy

// Alias para tipos comunes, asumiendo las definiciones de Lidar3D.h
using TShortArray = RoboCompLidar3D::TShortArray; // Usualmente vector<int16_t>
using TByteArray = RoboCompLidar3D::TByteArray;   // Usualmente vector<int8_t>
using TColorCloudData = RoboCompLidar3D::TColorCloudData;

class CloudCompressor {
private:
    // Evitamos copias y asignaciones dentro de los bucles de compresión
    // Usamos punteros a los datos de las estructuras para evitar el 'switch' dentro del bucle
    // y para poder pasar a funciones que manejan datos brutos.

    template<typename InputArray, typename DeltaType>
    static void encodeDelta(const InputArray& input, std::vector<DeltaType>& output) {
        output.resize(input.size());
        if (input.empty()) return;

        // Optimización: usar static_cast una sola vez y luego trabajar con tipos más grandes (32-bit o 16-bit)
        DeltaType prevVal = static_cast<DeltaType>(input[0]);
        output[0] = prevVal;

        for (size_t i = 1; i < input.size(); ++i) {
            DeltaType currentVal = static_cast<DeltaType>(input[i]);
            output[i] = currentVal - prevVal;
            prevVal = currentVal;
        }
    }

    template<typename DeltaType, typename OutputArray>
    static void decodeDelta(const std::vector<DeltaType>& deltas, OutputArray& output) {
        // Asume que output.resize(deltas.size()) se llama ANTES de llamar a decodeDelta
        if (deltas.empty()) return;

        DeltaType currentVal = deltas[0];
        output[0] = static_cast<typename OutputArray::value_type>(currentVal);

        for (size_t i = 1; i < deltas.size(); ++i) {
            currentVal += deltas[i];
            // Conversion y asignación
            output[i] = static_cast<typename OutputArray::value_type>(currentVal);
        }
    }

    // Un único método de compresión Zstd (más limpio y DRY)
    static void zstdCompress(const void* src, size_t srcSize, TByteArray& dest, int level) {
        // Evitamos el resize innecesario del vector dest a ZSTD_compressBound() si no es necesario.
        // Zstd puede escribir directamente en el buffer de destino.
        size_t bound = ZSTD_compressBound(srcSize);
        if (dest.size() < bound) {
             dest.resize(bound); // Solo se redimensiona si el tamaño actual es insuficiente.
        }
        size_t cSize = ZSTD_compress(dest.data(), dest.size(), src, srcSize, level);
        if (ZSTD_isError(cSize)) throw std::runtime_error(ZSTD_getErrorName(cSize));
        dest.resize(cSize); // Reducir al tamaño real
    }

    // Un único método de descompresión Zstd (más limpio y DRY)
    // Devuelve el tamaño real descomprimido
    static size_t zstdDecompress(const TByteArray& src, void* dest, size_t destSize) {
        size_t dSize = ZSTD_decompress(dest, destSize, src.data(), src.size());
        if (ZSTD_isError(dSize)) {
             throw std::runtime_error(std::string("ZSTD Decompress Error: ") + ZSTD_getErrorName(dSize));
        }
        return dSize;
    }


public:
    static void compress(TColorCloudData& in, int level = 1) {
        if (in.X.empty()) { 
            in.numberPoints = 0; 
        return; 
        }
        
        size_t n = in.numberPoints; // numberPoints ya debe estar establecid

        // ----------------------------------------------------
        // PRE-ASIGNACIÓN DE MEMORIA (para los buffers comprimidos)
        // ----------------------------------------------------
        size_t bound32 = ZSTD_compressBound(n * sizeof(int32_t));
        in.cX.resize(bound32);
        in.cY.resize(bound32);
        in.cZ.resize(bound32);
        
        // Los campos de color originales (R, G, B) serán los buffers comprimidos.
        size_t bound16 = ZSTD_compressBound(n * sizeof(int16_t));

        // Utilizamos un número de hilos igual al número de tareas (6) o menor si el hardware lo impone
        #pragma omp parallel num_threads(6)
        {
            // Buffers locales al hilo para la codificación delta
            std::vector<int32_t> deltaXYZ; 
            std::vector<int16_t> deltaRGB;
            
            // Asignación de tareas con #pragma omp sections (mejor para un número fijo y pequeño de tareas)
            #pragma omp sections nowait
            {
                // Tarea 1: X
                #pragma omp section
                {
                    encodeDelta(in.X, deltaXYZ);
                    in.X.clear(); in.X.shrink_to_fit();
                    zstdCompress(deltaXYZ.data(), deltaXYZ.size() * sizeof(int32_t), in.cX, level);
                }
                
                // Tarea 2: Y
                #pragma omp section
                {
                    encodeDelta(in.Y, deltaXYZ);
                    in.Y.clear(); in.Y.shrink_to_fit();
                    zstdCompress(deltaXYZ.data(), deltaXYZ.size() * sizeof(int32_t), in.cY, level);
                }
                
                // Tarea 3: Z
                #pragma omp section
                {
                    encodeDelta(in.Z, deltaXYZ);
                    in.Z.clear(); in.Z.shrink_to_fit();
                    zstdCompress(deltaXYZ.data(), deltaXYZ.size() * sizeof(int32_t), in.cZ, level);
                }
                
                // Tarea 4: R
                #pragma omp section
                {
                    encodeDelta(in.R, deltaRGB);
                    in.R.resize(bound16);
                    zstdCompress(deltaRGB.data(), deltaRGB.size() * sizeof(int16_t), in.R, level);
                }

                // Tarea 5: G
                #pragma omp section
                {
                    encodeDelta(in.G, deltaRGB);
                    in.G.resize(bound16);
                    zstdCompress(deltaRGB.data(), deltaRGB.size() * sizeof(int16_t), in.G, level);
                }

                // Tarea 6: B
                #pragma omp section
                {
                    encodeDelta(in.B, deltaRGB);
                    in.B.resize(bound16);
                    zstdCompress(deltaRGB.data(), deltaRGB.size() * sizeof(int16_t), in.B, level);
                }
            } // Fin sections
        } // Fin parallel
        in.compressed = true;
    }

    static void decompress(TColorCloudData& in) {

       size_t n = in.numberPoints;
        if (n == 0) return;

        // ----------------------------------------------------
        // 1. Pre-asignación para los buffers de salida descomprimidos (los 8MB)
        // ----------------------------------------------------
        in.X.resize(n); 
        in.Y.resize(n); 
        in.Z.resize(n);


        // ----------------------------------------------------
        // OpenMP para Descompresión (similar a Compresión)
        // ----------------------------------------------------
        #pragma omp parallel num_threads(6)
        {
            // Buffers de delta locales al hilo, redimensionados una sola vez
            // **Optimización:** Usamos std::array para un tamaño fijo y evitar la asignación del heap de vector<T>(n)
            // dentro del bucle. Sin embargo, dado que 'n' es variable, mejor mantener 'std::vector' con 'reserve' y 'resize'
            // fuera del bloque paralelo, o hacer un 'resize' local al hilo.
            std::vector<int32_t> deltaXYZ; 
            std::vector<int16_t> deltaRGB;
            
            deltaXYZ.resize(n);
            deltaRGB.resize(n);

            #pragma omp sections nowait
            {
                // Tarea 1: X
                #pragma omp section
                {
                    zstdDecompress(in.cX, deltaXYZ.data(), n * sizeof(int32_t));
                    in.cX.clear(); in.cX.shrink_to_fit();
                    decodeDelta(deltaXYZ, in.X);
                }
                
                // Tarea 2: Y
                #pragma omp section
                {
                    zstdDecompress(in.cY, deltaXYZ.data(), n * sizeof(int32_t));
                    in.cY.clear(); in.cY.shrink_to_fit();
                    decodeDelta(deltaXYZ, in.Y);
                }
                
                // Tarea 3: Z
                #pragma omp section
                {
                    zstdDecompress(in.cZ, deltaXYZ.data(), n * sizeof(int32_t));
                    in.cZ.clear(); in.cZ.shrink_to_fit();
                    decodeDelta(deltaXYZ, in.Z);
                }
                
                // Tarea 4: R
                #pragma omp section
                {
                    zstdDecompress(in.R, deltaRGB.data(), n * sizeof(int16_t));
                    in.R.resize(n);  
                    decodeDelta(deltaRGB, in.R);
                }

                // Tarea 5: G
                #pragma omp section
                {
                    zstdDecompress(in.G, deltaRGB.data(), n * sizeof(int16_t));
                    in.G.resize(n);  
                    decodeDelta(deltaRGB, in.G);
                }

                // Tarea 6: B
                #pragma omp section
                {
                    zstdDecompress(in.B, deltaRGB.data(), n * sizeof(int16_t));
                    in.B.resize(n);
                    decodeDelta(deltaRGB, in.B);
                }
            } // Fin sections
        } // Fin parallel


        in.compressed = false;
    }

};