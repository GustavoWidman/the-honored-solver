use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemFn, parse_macro_input};

#[proc_macro_attribute]
pub fn with_node(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);

    let vis = &input_fn.vis;
    let fn_name = &input_fn.sig.ident;
    let block = &input_fn.block;
    let asyncness = &input_fn.sig.asyncness;
    let output = &input_fn.sig.output;
    let inputs = &input_fn.sig.inputs;

    let expanded = if asyncness.is_some() {
        quote! {
            #vis async fn #fn_name(#inputs) #output {
                let ctx = r2r::Context::create()?;
                let mut node = r2r::Node::create(ctx, "the_honored_solver", "")?;

                let mut __node_handle: Option<tokio::task::JoinHandle<()>> = None;

                let mut start_node = |mut n: r2r::Node| {
                    let handle = tokio::task::spawn_blocking(move || loop {
                        n.spin_once(std::time::Duration::from_millis(100));
                    });
                    __node_handle = Some(handle);
                };

                let result = (|| async move { #block })().await;

                // if let Some(handle) = __node_handle {
                //     let _ = handle.await;
                // }

                result
            }
        }
    } else {
        quote! {
            #vis fn #fn_name(#inputs) #output {
                let ctx = r2r::Context::create()?;
                let mut node = r2r::Node::create(ctx, "the_honored_solver", "")?;

                let mut __node_handle: Option<std::thread::JoinHandle<()>> = None;

                let mut start_node = |mut n: r2r::Node| {
                    let handle = std::thread::spawn(move || loop {
                        n.spin_once(std::time::Duration::from_millis(100));
                    });
                    __node_handle = Some(handle);
                };

                let result = (|| { #block })();

                // if let Some(handle) = __node_handle {
                //     let _ = handle.join();
                // }

                result
            }
        }
    };

    TokenStream::from(expanded)
}
