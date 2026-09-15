#!/usr/bin/env python3
import pandas as pd
from pathlib import Path

csv_path = Path(__file__).parent / "metrics_output" / "metrics_summary.csv"

if not csv_path.exists():
    print(f"File non trovato: {csv_path}")
    exit(1)

df = pd.read_csv(csv_path)

# Se esiste tempi.csv, lo usiamo come "whitelist" (lista di esperimenti da includere)
tempi_csv_path = Path(__file__).parent / "tempi.csv"
if tempi_csv_path.exists():
    try:
        # Leggiamo ignorando le righe commentate con #
        df_tempi = pd.read_csv(tempi_csv_path, comment='#')
        # Filtriamo il dataframe originale tenendo solo le prove presenti in tempi.csv
        esperimenti_validi = df_tempi["experiment"].astype(str).tolist()
        df = df[df["experiment"].isin(esperimenti_validi)]
        print(f"Filtro applicato: considerati {len(esperimenti_validi)} esperimenti basati su tempi.csv")
    except Exception as e:
        print(f"[WARN] Non riesco a leggere tempi.csv per il filtro: {e}")

# Assegna la categoria in base al nome dell'esperimento
def get_category(name):
    name = str(name).lower()
    if name.startswith("base") or name in ["first", "second"]:
        return "1_Baseline"
    elif "caca" in name:
        return "2_Recovery_Drop (Z)"
    elif "swipe" in name:
        return "3_Recovery_Swipe (XY)"
    elif "completa" in name:
        return "4_Recovery_Completo (3D)"
    else:
        return "Altro"

df["Categoria"] = df["experiment"].apply(get_category)

# Aggrega i risultati
summary = df.groupby("Categoria").agg(
    Num_Prove=("experiment", "count"),
    RMSE_3D_Medio_m=("rmse_3d_m", "mean"),
    RMSE_3D_Minimo_m=("rmse_3d_m", "min"),
    Miglioramento_Post_Azione_Medio_perc=("improvement_%", lambda x: x.mean(skipna=True))
).reset_index()

# Arrotonda
summary["RMSE_3D_Medio_m"] = summary["RMSE_3D_Medio_m"].round(3)
summary["RMSE_3D_Minimo_m"] = summary["RMSE_3D_Minimo_m"].round(3)
summary["Miglioramento_Post_Azione_Medio_perc"] = summary["Miglioramento_Post_Azione_Medio_perc"].round(1)

print("\n=== TABELLA AGGREGATA PER ARTICOLO/TESI ===")
print(summary.to_markdown(index=False))

# Salva anche in markdown
out_md = Path(__file__).parent / "metrics_output" / "paper_table.md"
with open(out_md, "w") as f:
    f.write("# Risultati Aggregati (Baseline vs Recovery)\n\n")
    f.write(summary.to_markdown(index=False))
    f.write("\n")
print(f"\nTabella salvata in: {out_md}")
